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
import hashlib
import time

from .candidate_geometry import (
    copper_signature as _copper_signature,
    effective_clearance as _effective_clearance,
    path_blocker as _path_blocker,
    retain_identity_replacements as _retain_identity_replacements,
    segment_order_key as _segment_order_key)
from .context import PlannerContext
from .geometry import length, octolinear_paths, point_segment_distance
from .model import AddedSegment, GlossResult, Segment, Transformation, segment_key
from .pads import pad_contains, segment_hits_pad
from .statistics import classify_transformation
from .terminals import (find_pad_terminal_targets, find_track_terminal_targets,
                        movable_endpoint_pairs, vertex)
from .validation import validate_result


class PlanningDeadlineExceeded(RuntimeError):
    """Internal cooperative stop for the interactive planning budget."""


_INTERACTIVE_CANONICALIZATION_LIMIT = 512


def _check_deadline(deadline):
    if deadline is not None and time.monotonic() >= deadline:
        raise PlanningDeadlineExceeded(
            "Interactive planning time budget reached")


def _increment(counts, key):
    counts[key] = counts.get(key, 0) + 1


def _is_octolinear(a, b, tolerance=1e-6):
    """Return whether a segment follows a 0/45/90-degree direction."""
    dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
    return (dx <= tolerance or dy <= tolerance or
            abs(dx - dy) <= tolerance)


def _non_octolinear_count(segments):
    return sum(not _is_octolinear(
        (segment.start_x, segment.start_y),
        (segment.end_x, segment.end_y)) for segment in segments)


def _project_point_to_segment(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1e-18:
        return a
    ratio = ((point[0] - a[0]) * dx +
             (point[1] - a[1]) * dy) / denominator
    ratio = max(0.0, min(1.0, ratio))
    return (a[0] + ratio * dx, a[1] + ratio * dy)


def _custom_pad_event_paths(points, i, j, span, context, clearance):
    """Keep an existing lead-in/out and turn beside exact custom-pad copper."""
    if j - i < 2 or len({(round(item.width, 6),
                          round(item.clearance, 6)) for item in span}) != 1:
        return []
    start, end = points[i], points[j]
    first_next, last_previous = points[i + 1], points[j - 1]
    width = span[0].width
    candidates = []
    pads = context.nearby_pads(start, end, clearance, width)
    for region in pads:
        if not region.polygons:
            continue
        vertices = sorted({point for outer, holes in region.polygons
                           for polygon in (outer,) + tuple(holes)
                           for point in polygon})
        for vertex_point in vertices:
            lead = _project_point_to_segment(
                vertex_point, start, first_next)
            if (length(start, lead) > 1e-7 and
                    length(lead, first_next) > 1e-7):
                for tail in octolinear_paths(lead, end):
                    candidate = (start,) + tuple(tail)
                    if candidate not in candidates:
                        candidates.append(candidate)
            trail = _project_point_to_segment(
                vertex_point, last_previous, end)
            if (length(last_previous, trail) > 1e-7 and
                    length(trail, end) > 1e-7):
                for head in octolinear_paths(start, trail):
                    candidate = tuple(head) + (end,)
                    if candidate not in candidates:
                        candidates.append(candidate)
    return candidates


def _path_additions(path, originals, layer, net_id):
    """Map every original width run onto a movable replacement path.

    Width changes are electrical/routing properties, not fixed geometric
    anchors.  Their order and exact values are preserved, while their position
    is allowed to slide along the new octolinear path.
    """
    runs = []
    for segment in originals:
        segment_length = length(
            (segment.start_x, segment.start_y),
            (segment.end_x, segment.end_y))
        run = (segment.width, segment.clearance)
        run_key = (round(run[0], 6), round(run[1], 6))
        if (runs and
                (round(runs[-1][0][0], 6),
                 round(runs[-1][0][1], 6)) == run_key):
            runs[-1] = (runs[-1][0], runs[-1][1] + segment_length)
        else:
            runs.append((run, segment_length))
    if not runs:
        return []

    edge_lengths = [length(a, b) for a, b in zip(path, path[1:])]
    new_total = sum(edge_lengths)
    old_total = sum(run_length for _run, run_length in runs)
    if new_total <= 1e-9 or old_total <= 1e-9:
        return []

    transitions = []
    accumulated = 0.0
    for _run, run_length in runs[:-1]:
        accumulated += run_length
        transitions.append(new_total * accumulated / old_total)

    additions = []
    travelled = 0.0
    run_index = 0
    for a, b, edge_length in zip(path, path[1:], edge_lengths):
        if edge_length <= 1e-9:
            continue
        cuts = [travelled]
        cuts.extend(value for value in transitions
                    if travelled + 1e-9 < value < travelled + edge_length - 1e-9)
        cuts.append(travelled + edge_length)
        for start_distance, end_distance in zip(cuts, cuts[1:]):
            midpoint = (start_distance + end_distance) / 2.0
            while (run_index < len(transitions) and
                   midpoint > transitions[run_index] - 1e-9):
                run_index += 1
            local_start = (start_distance - travelled) / edge_length
            local_end = (end_distance - travelled) / edge_length
            start = a if local_start <= 1e-12 else (
                a[0] + (b[0] - a[0]) * local_start,
                a[1] + (b[1] - a[1]) * local_start)
            end = b if local_end >= 1.0 - 1e-12 else (
                a[0] + (b[0] - a[0]) * local_end,
                a[1] + (b[1] - a[1]) * local_end)
            if length(start, end) > 1e-9:
                additions.append(AddedSegment(
                    start, end, runs[run_index][0][0], layer, net_id,
                    runs[run_index][0][1]))
        travelled += edge_length
    return additions


def _endpoint_moved_into_pad(original, candidate, terminal, pad_targets):
    """Distinguish an actual pad contact from another movable termination."""
    if length(original, candidate) <= 1e-6:
        return False
    return any(pad_contains(region, candidate, tolerance=1e-6)
               for region in pad_targets.get(terminal, ()))


_SYNTHETIC_PREFIX = "__track_gloss__"
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
                moving.net_id, synthetic, False, False, moving.net_name,
                moving.clearance))
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


def _point_has_copper_terminal(model, segment, point, excluded_key):
    """Return whether the outer end still touches useful same-net copper."""
    for other in model.segments:
        if (segment_key(other) == excluded_key or
                other.net_id != segment.net_id or
                other.layer != segment.layer):
            continue
        if point_segment_distance(
                point, (other.start_x, other.start_y),
                (other.end_x, other.end_y)) <= (
                    segment.width + other.width) / 2.0 + 1e-6:
            return True
    return any(
        pad.net_id == segment.net_id and
        (not pad.layers or segment.layer in pad.layers) and
        segment_hits_pad(pad, point, point, margin=segment.width / 2.0 + 1e-6)
        for pad in model.pad_regions)


def _stub_pruning_contact(model, segment, point, excluded_key):
    """Classify the same-net contact that may make this a removable T tail.

    Preserve the original safe case (the tail terminates on wider through
    copper), and also accept the reported inverse-width case only when the
    narrower track itself terminates exactly at the T.  Merely touching the
    interior of a narrower track is not enough: accepting every same-net
    contact can recursively consume legitimate analytical split pieces.
    """
    inverse_width_endpoint = False
    for other in model.segments:
        if (segment_key(other) == excluded_key or
                other.net_id != segment.net_id or
                other.layer != segment.layer or
                point_segment_distance(
                    point, (other.start_x, other.start_y),
                    (other.end_x, other.end_y)) > 1e-6):
            continue
        if other.width > segment.width + 1e-6:
            return "wider_through_track"
        if (other.width < segment.width - 1e-6 and
                vertex(*point) in {
                    vertex(other.start_x, other.start_y),
                    vertex(other.end_x, other.end_y)}):
            inverse_width_endpoint = True
    return "narrower_endpoint" if inverse_width_endpoint else None


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
                contact = _stub_pruning_contact(
                    current_model, segment, junction, key)
                outer_terminal = _point_is_terminal(
                    current_model, segment, outer, key)
                if contact == "narrower_endpoint":
                    # The inverse-width case needs the stronger copper-area
                    # check; otherwise nearby useful copper can look detached
                    # merely because its centrelines do not meet exactly.
                    outer_terminal = _point_has_copper_terminal(
                        current_model, segment, outer, key)
                if (outer_count == 1 and not outer_terminal and contact and
                        (junction_count >= 3 or junction_crossing)):
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
    occupied_keys = {segment_key(segment) for segment in segments}
    for index, addition in enumerate(plan.additions):
        key = "{}pass-{}-{}".format(_SYNTHETIC_PREFIX, pass_index, index)
        collision = 0
        while key in occupied_keys:
            collision += 1
            key = "{}pass-{}-{}-{}".format(
                _SYNTHETIC_PREFIX, pass_index, index, collision)
        occupied_keys.add(key)
        # KiCad stores board coordinates as integer nanometres. Quantize every
        # in-memory replacement at the same boundary so a reported fixed point
        # remains a fixed point after BoardAdapter applies and saves it.
        start_x, start_y = (round(value, 6) for value in addition.start)
        end_x, end_y = (round(value, 6) for value in addition.end)
        segments.append(Segment(
            start_x, start_y, end_x, end_y,
            round(addition.width, 6), addition.layer, addition.net_id,
            key, False, False, names.get(addition.net_id, ""),
            addition.clearance))
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
                        (first.net_id, first.layer, round(first.width, 6),
                         round(first.clearance, 6)) !=
                        (second.net_id, second.layer, round(second.width, 6),
                         round(second.clearance, 6))):
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
                    key, False, False, first.net_name, first.clearance))
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
                segment.layer, segment.net_id, segment.clearance))
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
    result.angle_corrections = sum(plan.angle_corrections for plan in passes)
    for plan in passes:
        result.warnings.extend(plan.warnings)
        result.transformations.extend(plan.transformations)
        for key, value in plan.search_counts.items():
            result.search_counts[key] = result.search_counts.get(key, 0) + value
        for key, value in plan.blocking_nets.items():
            result.blocking_nets[key] = result.blocking_nets.get(key, 0) + value
    for stub in pruned_stubs:
        stub_mm = length((stub.start_x, stub.start_y),
                         (stub.end_x, stub.end_y))
        result.transformations.append(Transformation(
            "track_slide", "segment_simplification", stub.net_id,
            stub.net_name, stub.layer, stub.width, stub_mm, 0.0, 1, 0))
    _retain_identity_replacements(original_model, result)
    context = PlannerContext(original_model)
    immutable_cover_keys = set(context.segment_by_key) - set(original_eligible)
    replaced_keys = set(result.remove_keys)
    for index, addition in enumerate(result.additions):
        validation_clearance = addition.clearance
        if validation_clearance < 0.0:
            validation_clearance = max(
                [_effective_clearance(original_model, segment)
                 for segment in original_model.segments
                 if segment_key(segment) in original_eligible and
                 segment.net_id == addition.net_id and
                 segment.layer == addition.layer] +
                [original_model.minimum_clearance])
        moving = Segment(
            addition.start[0], addition.start[1],
            addition.end[0], addition.end[1], addition.width,
            addition.layer, addition.net_id, "composed:{}".format(index),
            clearance=validation_clearance)
        # Preserve original breakpoints while validating a collinearly merged
        # result.  This lets unchanged copper retain its pre-existing status
        # without exempting the genuinely new portion of the same addition.
        ax, ay = addition.start
        bx, by = addition.end
        denominator = (bx - ax) ** 2 + (by - ay) ** 2
        cuts = [(0.0, addition.start), (1.0, addition.end)]
        if denominator > 0.0:
            for key in replaced_keys:
                existing = context.segment_by_key.get(key)
                if existing is None:
                    continue
                for point in ((existing.start_x, existing.start_y),
                              (existing.end_x, existing.end_y)):
                    if point_segment_distance(point, addition.start,
                                              addition.end) > 1e-6:
                        continue
                    parameter = ((point[0] - ax) * (bx - ax) +
                                 (point[1] - ay) * (by - ay)) / denominator
                    if 1e-9 < parameter < 1.0 - 1e-9:
                        cuts.append((parameter, point))
        path = [point for _parameter, point in sorted(set(cuts))]
        blocker = _path_blocker(
            original_model, path, moving,
            replaced_keys, original_model.minimum_clearance, context,
            immutable_cover_keys)
        if blocker:
            raise ValueError(
                "Composed candidate violates {}".format(blocker[0]))
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
    refined = _compose_refined_plan(
        model, eligible, simulated, current_eligible, passes, pruned_stubs)

    def objective(plan):
        return (
            plan.angle_corrections, round(plan.saved_mm, 9),
            len(plan.remove_keys) - len(plan.additions),
            -len(plan.additions))

    # A newly available safe candidate can alter a later refinement pass, but
    # it must never make the final answer worse than the already-valid initial
    # optimum. This also makes relaxing an over-conservative rule monotonic.
    return refined if objective(refined) > objective(initial) else initial


def smooth_selected_chains(model, eligible_segment_keys, *, min_gain=0.01,
                           allow_equal_length_simpler=False, clearance=0.1,
                           equal_length_tolerance=0.001,
                           span_strategy="farthest", path_preference=0,
                           collect_statistics=True, planner_context=None,
                           deadline=None):
    """Return an immutable edit plan; never mutates ``model``.

    Original octolinear smoothing algorithm: KiCadRoutingTools, DrAndyHaas.
    Standalone adaptation: only explicit KIID/geometric keys enter a chain.
    Non-selected copper remains in incidence and obstacle calculations.
    """
    eligible = {str(key) for key in eligible_segment_keys}
    planner_context = (planner_context if planner_context is not None and
                       planner_context.model is model else PlannerContext(model))
    immutable_cover_keys = set(planner_context.segment_by_key) - eligible
    result = GlossResult()
    net_names = {segment.net_id: segment.net_name
                 for segment in model.segments if segment.net_id > 0}
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
            groups[(s.net_id, s.layer)].append(s)

    for net_id, layer in sorted(groups):
        _check_deadline(deadline)
        candidates = sorted(groups[(net_id, layer)], key=_segment_order_key)
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
            touching.sort(key=_segment_order_key)
        anchors = sorted(v for v in adjacency if not interior(v))
        used = set()
        for anchor in anchors:
            _check_deadline(deadline)
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
                    nxt = sorted(
                        (s for s in adjacency[current]
                         if segment_key(s) not in used),
                        key=_segment_order_key)
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
                    _check_deadline(deadline)
                    choices = []
                    for j in range(len(chain), i, -1):
                        _check_deadline(deadline)
                        old_len = cumulative[j] - cumulative[i]
                        span = chain[i:j]
                        angle_corrections = _non_octolinear_count(span)
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
                        if (j == i + 1 and not angle_corrections and not
                                (start_targets or end_targets or
                                 start_pads or end_pads)):
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
                        for path in _custom_pad_event_paths(
                                points, i, j, span, planner_context,
                                clearance):
                            if path not in paths:
                                paths.append(path)
                        if path_preference:
                            paths.reverse()
                        for path in paths:
                            _check_deadline(deadline)
                            if collect_statistics:
                                _increment(result.search_counts, "paths_evaluated")
                            new_len = sum(length(a, b) for a, b in zip(path, path[1:]))
                            additions = _path_additions(
                                path, span, layer, net_id)
                            new_count = len(additions)
                            gain = old_len - new_len
                            acceptable = angle_corrections > 0 or gain >= min_gain or (
                                allow_equal_length_simpler and abs(gain) <= equal_length_tolerance
                                and new_count < j - i)
                            if not acceptable:
                                if collect_statistics:
                                    _increment(result.search_counts, "not_improving")
                                continue
                            blocker = None
                            for addition in additions:
                                moving = replace(chain[i], width=addition.width)
                                blocker = _path_blocker(
                                    model, (addition.start, addition.end),
                                    moving, replaced, clearance, planner_context,
                                    immutable_cover_keys)
                                if blocker:
                                    break
                            if blocker:
                                if collect_statistics:
                                    reason, blocker_net_id = blocker
                                    _increment(result.search_counts, reason)
                                    if blocker_net_id > 0:
                                        label = (net_names.get(blocker_net_id) or
                                                 "net {}".format(blocker_net_id))
                                        _increment(result.blocking_nets, label)
                                continue
                            if collect_statistics:
                                _increment(result.search_counts, "accepted_options")
                            choices.append((j, path, gain, replaced,
                                            new_count, old_len,
                                            angle_corrections, additions))
                        if choices and span_strategy == "farthest":
                            break
                    return choices

                spans = {}
                if span_strategy == "global":
                    # Weighted interval scheduling: maximize total saved length
                    # over the WHOLE chain, then segment reduction. This avoids
                    # the A-before-B bias of greedy shortcut acceptance.
                    options = {i: choices_at(i) for i in range(len(chain))}
                    best = {len(chain): (0, 0.0, 0, [])}
                    for i in range(len(chain) - 1, -1, -1):
                        skip = best.get(i + 1, (0, 0.0, 0, []))
                        candidates_dp = [skip]
                        for choice in options.get(i, ()):
                            (j, path, gain, replaced, new_count, _old_len,
                             corrections, _additions) = choice
                            tail = best.get(j, (0, 0.0, 0, []))
                            reduction = (j - i) - new_count
                            candidates_dp.append((
                                corrections + tail[0], gain + tail[1],
                                reduction + tail[2], [(i, choice)] + tail[3]))

                        def dp_key(candidate):
                            signature = tuple((start, selected[0], tuple(selected[1]))
                                              for start, selected in candidate[3])
                            return (candidate[0], round(candidate[1], 9), candidate[2],
                                    tuple((-a, -b, path) for a, b, path in signature))

                        best[i] = max(candidates_dp, key=dp_key)
                    spans = {start: selected for start, selected in best[0][3]}
                else:
                    i = 0
                    while i < len(chain):
                        choices = choices_at(i)
                        if choices:
                            if span_strategy == "max_gain":
                                choices.sort(key=lambda c: (-c[6], -c[2], c[4],
                                                           -c[0], tuple(c[1])))
                            elif span_strategy == "farthest":
                                choices.sort(key=lambda c: (-c[6], -c[0], -c[2],
                                                           c[4], tuple(c[1])))
                            found = choices[0]
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
                    (j, path, gain, replaced, _new_count, _old_len,
                     corrections, additions) = spans[k]
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
                            chain[k:j], path, mechanism, equal_length_tolerance,
                            after_segments=len(additions)))
                    result.remove_keys.extend(segment_key(s) for s in chain[k:j])
                    result.additions.extend(additions)
                    result.saved_mm += gain
                    result.angle_corrections += corrections
                    chain_changed = True
                    k = j
                if chain_changed:
                    result.chains_changed += 1

    result.saved_mm = max(0.0, result.saved_mm)
    _retain_identity_replacements(model, result)
    validate_result(model, eligible, result)
    return result


_MAX_LOCAL_JUNCTION_SCOPES = 4
_LOCAL_JUNCTION_ELIGIBLE_LIMIT = 16


def _junction_branch_scopes(model, eligible_segment_keys,
                            limit=_MAX_LOCAL_JUNCTION_SCOPES):
    """Return deterministic proper branch scopes around selected junctions.

    Only endpoint incidence is used here, matching chain construction. Each
    walk moves away from a junction through pure degree-two continuation and
    stops at the next anchor. The hard cap prevents whole-board selections
    from turning into a combinatorial subset search.
    """
    eligible = set(eligible_segment_keys)
    if len(eligible) < 2 or len(eligible) > _LOCAL_JUNCTION_ELIGIBLE_LIMIT:
        return ()
    selected = [segment for segment in model.segments
                if segment_key(segment) in eligible and
                not segment.locked and not segment.arc]
    selected_by_key = {segment_key(segment): segment for segment in selected}
    selected_incidence = defaultdict(list)
    all_incidence = defaultdict(int)
    for segment in model.segments:
        for point in (vertex(segment.start_x, segment.start_y),
                      vertex(segment.end_x, segment.end_y)):
            all_incidence[(segment.net_id, point)] += 1
    for segment in selected:
        for point in (vertex(segment.start_x, segment.start_y),
                      vertex(segment.end_x, segment.end_y)):
            selected_incidence[(segment.net_id, segment.layer, point)].append(
                segment_key(segment))

    scopes = set()
    junctions = sorted(
        (node, tuple(sorted(keys)))
        for node, keys in selected_incidence.items()
        if len(keys) >= 2 and all_incidence[(node[0], node[2])] >= 3)
    for (net_id, layer, junction), incident_keys in junctions:
        for first_key in incident_keys:
            branch = set()
            previous = junction
            current_key = first_key
            while current_key not in branch:
                branch.add(current_key)
                segment = selected_by_key[current_key]
                start = vertex(segment.start_x, segment.start_y)
                end = vertex(segment.end_x, segment.end_y)
                current = end if start == previous else start
                node = (net_id, layer, current)
                continuations = [
                    key for key in selected_incidence.get(node, ())
                    if key != current_key and key not in branch]
                if (all_incidence[(net_id, current)] != 2 or
                        len(continuations) != 1):
                    break
                previous, current_key = current, continuations[0]
            frozen = frozenset(branch)
            if frozen and frozen != frozenset(eligible):
                scopes.add(frozen)
    return tuple(sorted(scopes, key=lambda scope: (
        -len(scope), tuple(sorted(scope)))))[:max(0, int(limit))]


def generate_candidate_plans(model, eligible_segment_keys, **kwargs):
    """Generate deterministic global and isolated-group fallback plans."""
    deadline = kwargs.get("deadline")
    cancellation_grace_seconds = kwargs.pop(
        "cancellation_grace_seconds", 1.0)
    _check_deadline(deadline)
    max_refinement_passes = kwargs.pop("max_refinement_passes", 3)
    parallel = kwargs.pop("parallel", False)
    converge_groups = kwargs.pop("converge_groups", False)
    group_max_passes = kwargs.pop("group_max_passes", 6)
    allow_netclass_seed = kwargs.pop("_allow_netclass_seed", True)
    allow_junction_scopes = kwargs.pop(
        "_allow_junction_scopes",
        len(set(eligible_segment_keys)) <= _LOCAL_JUNCTION_ELIGIBLE_LIMIT)
    kwargs.setdefault("planner_context", PlannerContext(model))
    plans = []
    seen = set()
    rejected = []

    known_segments = {segment_key(segment): segment
                      for segment in model.segments}

    def removal_signature(plan):
        return tuple(sorted(
            _segment_order_key(known_segments[key])[:-1]
            for key in set(plan.remove_keys) if key in known_segments))

    def measured_gain(plan):
        removed_mm = sum(
            length((known_segments[key].start_x, known_segments[key].start_y),
                   (known_segments[key].end_x, known_segments[key].end_y))
            for key in set(plan.remove_keys) if key in known_segments)
        added_mm = sum(length(addition.start, addition.end)
                       for addition in plan.additions)
        return max(0.0, removed_mm - added_mm)

    def add_plan(plan, already_validated=False):
        # Every route into the candidate pool uses the same complete safety
        # gate. In particular, independently safe width/net plans are not
        # necessarily safe after being combined.
        if not already_validated:
            validate_result(model, set(eligible_segment_keys), plan,
                            check_connectivity=True)
        plan.saved_mm = measured_gain(plan)
        signature = (
            removal_signature(plan),
            tuple(sorted((a.start, a.end, a.width, a.layer, a.net_id)
                         for a in plan.additions)),
        )
        if signature not in seen:
            seen.add(signature)
            plans.append(plan)

    eligible = set(eligible_segment_keys)
    groups = defaultdict(set)
    primary_groups = defaultdict(set)
    for segment in model.segments:
        key = segment_key(segment)
        if key in eligible:
            primary_groups[(segment.net_id, segment.layer)].add(key)
            groups[(segment.net_id, segment.layer,
                    round(segment.width, 6))].add(key)
    group_keys = sorted(groups) if len(groups) > 1 else []
    primary_keys = sorted(primary_groups)
    parallel_rows = None
    parallel_timed_out = False
    # Process startup is intentionally reserved for substantial selections;
    # ordinary one-connection glosses stay in-process and avoid IPC latency.
    # The historical offline first pass is the whole-scope global search. Its
    # basin is part of the frozen CLI/plugin equivalence contract; parallel
    # workers join later opening passes. Bounded interactive work parallelizes
    # immediately because responsiveness takes priority over exhausting that
    # first global basin.
    if (parallel and (deadline is not None or converge_groups) and
            len(eligible) >= 64 and
            len(primary_keys) + len(group_keys) > 1):
        from .parallel import run_parallel_group_plans
        tasks = [(('primary',) + key,
                  tuple(sorted(primary_groups[key]))) for key in primary_keys]
        # A single-width net/layer fallback is byte-for-byte the same search as
        # its primary task. Reuse that result instead of sending duplicate work
        # to a subprocess; only mixed-width subgroups need separate searches.
        tasks.extend(
            (('fallback',) + key, tuple(sorted(groups[key])))
            for key in group_keys
            if groups[key] != primary_groups[key[:2]])
        parallel_kwargs = dict(kwargs)
        if converge_groups:
            parallel_kwargs["_allow_junction_scopes"] = allow_junction_scopes
        remaining = (None if deadline is None else
                     max(0.0, deadline - time.monotonic()))
        # Keep a deterministic fraction of the interactive budget for merging
        # and validating completed groups. Spending the entire deadline inside
        # workers would leave no safe composed plan to return.
        worker_timeout = (None if remaining is None else remaining * 0.75)
        parallel_result = run_parallel_group_plans(
            model, tasks, parallel_kwargs, converge=converge_groups,
            max_passes=group_max_passes, timeout_seconds=worker_timeout,
            cancellation_grace_seconds=cancellation_grace_seconds)
        if parallel_result is not None:
            parallel_rows, parallel_timed_out = parallel_result
        if parallel_timed_out:
            rejected.append(
                "Interactive planning time budget reached; retained {} "
                "completed group result(s)".format(len(parallel_rows)))
        if parallel_rows is not None and any(row[2] for row in parallel_rows):
            rejected.extend(row[2] for row in parallel_rows if row[2])
        if (parallel_rows is not None and
                not any(row[1] is not None for row in parallel_rows)):
            rejected.append(
                "Parallel workers returned no usable group; sequential "
                "planning fallback used")
            parallel_rows = None

    def merge(selected):
        merged = GlossResult()
        for plan in selected:
            merged.remove_keys.extend(plan.remove_keys)
            merged.additions.extend(plan.additions)
            merged.chains_considered += plan.chains_considered
            merged.chains_changed += plan.chains_changed
            merged.angle_corrections += plan.angle_corrections
            merged.warnings.extend(plan.warnings)
            merged.transformations.extend(plan.transformations)
            for key, value in plan.search_counts.items():
                merged.search_counts[key] = merged.search_counts.get(key, 0) + value
            for key, value in plan.blocking_nets.items():
                merged.blocking_nets[key] = merged.blocking_nets.get(key, 0) + value
        merged.saved_mm = measured_gain(merged)
        return merged

    # Global weighted interval scheduling now evaluates every endpoint and
    # elbow candidate deterministically, so the older greedy/path-order passes
    # cannot improve its objective and only multiply runtime.
    if parallel_rows is not None:
        primary_results = {row[0][1:]: row[1] for row in parallel_rows
                           if row[0][0] == 'primary' and row[1] is not None}
        ranked_primary = sorted(
            ((key, primary_results[key]) for key in primary_keys
             if primary_results.get(key) is not None and
             primary_results[key].changed),
            key=lambda row: (
                -row[1].angle_corrections,
                -round(row[1].saved_mm, 9),
                -(len(row[1].remove_keys) - len(row[1].additions)),
                row[0]))
        completed_primary_keys = [
            key for key in primary_keys if key in primary_results]
        if completed_primary_keys and deadline is None:
            try:
                add_plan(merge([
                    primary_results[key] for key in completed_primary_keys]))
            except ValueError as error:
                rejected.append(str(error))

        # Locally converged groups can expose more than one simultaneous
        # cross-net conflict, so leave-one-out is not sufficient. Build a
        # deterministic maximal safe batch by adding gain-ranked independent
        # groups only while the complete connectivity/clearance gate passes.
        selected_primary = []
        for _key, plan in ranked_primary:
            if deadline is not None and time.monotonic() >= deadline:
                rejected.append(
                    "Interactive planning time budget reached during plan composition")
                break
            candidate = merge(selected_primary + [plan])
            try:
                validate_result(model, eligible, candidate,
                                check_connectivity=True)
            except ValueError as error:
                rejected.append(str(error))
                continue
            selected_primary.append(plan)
        if selected_primary:
            add_plan(merge(selected_primary), already_validated=True)
        for _key, plan in ranked_primary:
            if deadline is not None and time.monotonic() >= deadline:
                break
            add_plan(plan, already_validated=True)
        if deadline is None:
            # Offline/CLI quality remains anchored to the historical global
            # search result. Parallel groups add candidates and throughput but
            # must never replace a better whole-scope basin merely because the
            # old worker failure used to fall back to this search implicitly.
            try:
                add_plan(smooth_selected_chains(
                    model, eligible_segment_keys, span_strategy="global",
                    path_preference=0, **kwargs), already_validated=True)
            except ValueError as error:
                rejected.append(str(error))
    else:
        try:
            add_plan(smooth_selected_chains(
                model, eligible_segment_keys, span_strategy="global",
                path_preference=0, **kwargs), already_validated=True)
        except ValueError as error:
            rejected.append(str(error))

    # Eligibility is authorization, not an obligation to move every selected
    # segment. At a selected T, treating every authorized branch as mutable can
    # remove a sliding target and make a larger selection worse than a smaller
    # one. Preserve a bounded set of branch-local alternatives, with all other
    # authorized copper temporarily immutable for that candidate.
    if allow_junction_scopes:
        for branch_scope in _junction_branch_scopes(model, eligible):
            branch_kwargs = dict(kwargs)
            branch_kwargs.pop("planner_context", None)
            try:
                branch_plan = generate_converged_plan(
                    model, branch_scope,
                    max_passes=max(1, min(group_max_passes, 4)),
                    return_partial_on_limit=True,
                    batch_group_convergence=False,
                    parallel=False,
                    _allow_junction_scopes=False,
                    **branch_kwargs)
                if branch_plan.changed:
                    add_plan(branch_plan)
            except ValueError as error:
                rejected.append(str(error))

    # Batch fallback pool. If KiCad rejects the combined optimum, try leaving
    # out one independently scoped group, then each group alone. This keeps one
    # difficult track from blocking every other selected track while retaining
    # deterministic, gain-ranked behavior.
    group_plans = []
    # With a single group the global plan above is exactly the same call.  Do
    # not repeat an expensive no-op search (formerly visible as a GUI freeze
    # on dense tuned connections).
    if parallel_rows is not None:
        primary_results = {row[0][1:]: row[1] for row in parallel_rows
                           if row[0][0] == 'primary'}
        for group_key, plan, _error in parallel_rows:
            if (group_key[0] == 'fallback' and plan is not None and
                    plan.changed):
                group_plans.append(plan)
        for group_key in group_keys:
            if groups[group_key] == primary_groups[group_key[:2]]:
                plan = primary_results.get(group_key[:2])
                if plan is not None and plan.changed:
                    group_plans.append(plan)
    else:
        for group_key in group_keys:
            try:
                plan = smooth_selected_chains(
                    model, groups[group_key], span_strategy="global",
                    path_preference=0, **kwargs)
                if plan.changed:
                    group_plans.append(plan)
            except ValueError as error:
                rejected.append(str(error))

    if len(group_plans) > 1 and deadline is None:
        merged_fallback_valid = False
        try:
            add_plan(merge(group_plans))
            merged_fallback_valid = True
        except ValueError as error:
            rejected.append(str(error))
        # Leave-one-out plans only add value when the full combination is
        # unsafe. Avoid O(group_count) whole-board connectivity validations on
        # the normal path where all independent groups combine successfully.
        if not merged_fallback_valid:
            for omitted in range(len(group_plans)):
                try:
                    add_plan(merge([p for index, p in enumerate(group_plans)
                                    if index != omitted]))
                except ValueError as error:
                    rejected.append(str(error))
    for plan in group_plans:
        if deadline is not None and time.monotonic() >= deadline:
            rejected.append(
                "Interactive planning time budget reached during fallback composition")
            break
        try:
            add_plan(plan, already_validated=True)
        except ValueError as error:
            rejected.append(str(error))
    plans.sort(key=lambda p: (
        -p.angle_corrections, -round(p.saved_mm, 9),
        -(len(p.remove_keys) - len(p.additions)), len(p.additions),
        removal_signature(p)))
    if not plans:
        plans.append(GlossResult(warnings=sorted(set(rejected))))
    elif parallel_timed_out:
        plans[0].warnings.append(
            "Interactive planning time budget reached; a validated partial "
            "group plan was retained.")
    elif (plans[0].changed and max_refinement_passes and
          (deadline is None or time.monotonic() < deadline) and
          len(set(eligible_segment_keys)) <= _REFINEMENT_SCOPE_LIMIT):
        try:
            plans[0] = _refine_plan(
                model, eligible_segment_keys, plans[0], dict(kwargs),
                max_refinement_passes)
            plans.sort(key=lambda p: (
                -p.angle_corrections, -round(p.saved_mm, 9),
                -(len(p.remove_keys) - len(p.additions)), len(p.additions),
                removal_signature(p)))
        except ValueError as error:
            plans[0].warnings.append("Refinement rejected: " + str(error))

    # With mixed widths, relaxing a clearance can expose a different first
    # refinement and occasionally lead the local search to a worse basin. A
    # second, more conservative netclass seed preserves the former safe
    # solution as a candidate. It is intentionally limited to mixed-width
    # scopes, where width-run topology makes this useful, and cannot recurse.
    selected_segments = [
        segment for segment in model.segments
        if segment_key(segment) in eligible]
    widths_by_group = defaultdict(set)
    for segment in selected_segments:
        widths_by_group[(segment.net_id, segment.layer)].add(
            round(segment.width, 6))
    mixed_width = any(len(widths) > 1
                      for widths in widths_by_group.values())
    resolved_differs = any(
        segment.clearance >= 0.0 and
        abs(segment.clearance - max(
            model.minimum_clearance,
            model.net_clearances.get(segment.net_id, 0.0))) > 1e-9
        for segment in selected_segments)
    if (allow_netclass_seed and mixed_width and resolved_differs and
            (deadline is None or time.monotonic() < deadline)):
        fallback_model = replace(
            model, segments=[replace(segment, clearance=-1.0)
                             for segment in model.segments])
        fallback_kwargs = dict(kwargs)
        fallback_kwargs.pop("planner_context", None)
        try:
            fallback = generate_candidate_plans(
                fallback_model, eligible_segment_keys,
                max_refinement_passes=max_refinement_passes,
                parallel=False, _allow_netclass_seed=False,
                **fallback_kwargs)[0]
            if fallback.changed:
                add_plan(fallback)
        except ValueError as error:
            rejected.append(str(error))
        plans.sort(key=lambda p: (
            -p.angle_corrections, -round(p.saved_mm, 9),
            -(len(p.remove_keys) - len(p.additions)), len(p.additions),
            removal_signature(p)))
    return plans


def _model_geometry_signature(model):
    """Stable identity-free route signature used by fixed-point search."""
    rows = []
    for segment in model.segments:
        start = (round(segment.start_x, 6), round(segment.start_y, 6))
        end = (round(segment.end_x, 6), round(segment.end_y, 6))
        first, second = sorted((start, end))
        rows.append((first, second, round(segment.width, 6), segment.layer,
                     segment.net_id, bool(segment.locked), bool(segment.arc)))
    return tuple(sorted(rows))


def _canonicalize_model_segments(model):
    """Remove pcbnew/serialization order from every planning decision."""
    ordered = sorted(model.segments, key=_segment_order_key)
    return replace(model, segments=ordered)


def _canonicalize_eligible_subdivisions(model, eligible):
    """Merge collinear eligible input pieces for subdivision-invariant search.

    The returned expansion map lets the caller restore every canonical segment
    that survives planning unchanged.  Consequently artificial input
    breakpoints cannot alter candidate generation, while an untouched route is
    not gratuitously rewritten on the live board.
    """
    segments = list(model.segments)
    eligible = set(eligible)
    expansions = {}
    counter = 0
    while True:
        merged = None
        for index, first in enumerate(segments):
            first_key = segment_key(first)
            if first_key not in eligible or first.locked or first.arc:
                continue
            first_ends = ((first.start_x, first.start_y),
                          (first.end_x, first.end_y))
            for second in segments[index + 1:]:
                second_key = segment_key(second)
                if (second_key not in eligible or second.locked or second.arc or
                        (first.net_id, first.layer, round(first.width, 6),
                         round(first.clearance, 6)) !=
                        (second.net_id, second.layer, round(second.width, 6),
                         round(second.clearance, 6))):
                    continue
                second_ends = ((second.start_x, second.start_y),
                               (second.end_x, second.end_y))
                shared = set(first_ends) & set(second_ends)
                if len(shared) != 1:
                    continue
                joint = next(iter(shared))
                touching_branch = any(
                    candidate is not first and candidate is not second and
                    candidate.net_id == first.net_id and
                    candidate.layer == first.layer and
                    point_segment_distance(
                        joint,
                        (candidate.start_x, candidate.start_y),
                        (candidate.end_x, candidate.end_y)) <= 1e-7
                    for candidate in segments)
                touching_anchor = any(
                    obstacle.net_id == first.net_id and
                    (not obstacle.layers or first.layer in obstacle.layers) and
                    ((joint[0] - obstacle.x) ** 2 +
                     (joint[1] - obstacle.y) ** 2) ** 0.5 <=
                    obstacle.radius + 1e-7
                    for obstacle in model.obstacles)
                touching_anchor = touching_anchor or any(
                    pad.net_id == first.net_id and
                    (not pad.layers or first.layer in pad.layers) and
                    pad_contains(pad, joint)
                    for pad in model.pad_regions)
                if touching_branch or touching_anchor:
                    continue
                outer_first = first_ends[1] if first_ends[0] == joint else first_ends[0]
                outer_second = second_ends[1] if second_ends[0] == joint else second_ends[0]
                cross = ((joint[0] - outer_first[0]) *
                         (outer_second[1] - joint[1]) -
                         (joint[1] - outer_first[1]) *
                         (outer_second[0] - joint[0]))
                if abs(cross) > 1e-7 or outer_first == outer_second:
                    continue
                key = "{}input-{}".format(_SYNTHETIC_PREFIX, counter)
                counter += 1
                originals = (expansions.pop(first_key, [first]) +
                             expansions.pop(second_key, [second]))
                replacement = Segment(
                    outer_first[0], outer_first[1],
                    outer_second[0], outer_second[1], first.width,
                    first.layer, first.net_id, key, False, False,
                    first.net_name, first.clearance)
                merged = (first, second, replacement, originals)
                break
            if merged is not None:
                break
        if merged is None:
            break
        first, second, replacement, originals = merged
        segments.remove(first)
        segments.remove(second)
        eligible.discard(segment_key(first))
        eligible.discard(segment_key(second))
        segments.append(replacement)
        eligible.add(segment_key(replacement))
        expansions[segment_key(replacement)] = originals
    return replace(model, segments=segments), eligible, expansions


def _restore_unchanged_subdivisions(model, eligible, expansions):
    """Expand canonical input segments which survived the search unchanged."""
    segments = []
    eligible = set(eligible)
    for segment in model.segments:
        key = segment_key(segment)
        originals = expansions.get(key)
        if originals is None:
            segments.append(segment)
            continue
        segments.extend(originals)
        eligible.discard(key)
        eligible.update(segment_key(original) for original in originals)
    return replace(model, segments=segments), eligible


def _convergence_state(model, pass_index, initial_mm, previous_mm, event,
                       step=None):
    """Return a compact, identity-free diagnostic for one global pass."""
    straight = [segment for segment in model.segments if not segment.arc]
    current_mm = sum(length(
        (segment.start_x, segment.start_y),
        (segment.end_x, segment.end_y)) for segment in straight)
    signature = _model_geometry_signature(model)
    gain_mm = previous_mm - current_mm
    result = {
        "event": event,
        "pass": pass_index,
        "copper_mm": current_mm,
        "segments": len(straight),
        "pass_gain_mm": gain_mm,
        "pass_gain_percent": (
            100.0 * gain_mm / previous_mm if previous_mm else 0.0),
        "cumulative_gain_mm": initial_mm - current_mm,
        "cumulative_gain_percent": (
            100.0 * (initial_mm - current_mm) / initial_mm
            if initial_mm else 0.0),
        "geometry_signature": hashlib.sha256(
            repr(signature).encode("utf-8")).hexdigest()[:16],
    }
    if step is not None:
        result.update({
            "removed": len(step.remove_keys),
            "added": len(step.additions),
            "changed_nets": sorted({
                transformation.net_name
                for transformation in step.transformations}),
        })
    return result


def generate_converged_plan(model, eligible_segment_keys, *, max_passes=16,
                            pass_observer=None, return_partial_on_limit=False,
                            batch_group_convergence=True,
                            **kwargs):
    """Compose repeated global plans into one validated fixed-point edit.

    The authorized scope never grows: replacement segments inherit eligibility,
    while immutable copper remains an obstacle or a sliding termination target.
    The returned plan always addresses ``model`` directly, so KiCad can apply it
    as one operation and preserve a single Undo step.
    """
    if max_passes < 1:
        raise ValueError("max_passes must be at least one")
    deadline = kwargs.get("deadline")
    _check_deadline(deadline)
    source_model = _canonicalize_model_segments(model)
    source_eligible = set(eligible_segment_keys)
    subdivision_canonicalization_skipped = (
        deadline is not None and
        len(source_eligible) > _INTERACTIVE_CANONICALIZATION_LIMIT)
    if subdivision_canonicalization_skipped:
        model, original_eligible, input_expansions = (
            source_model, set(source_eligible), {})
    else:
        model, original_eligible, input_expansions = \
            _canonicalize_eligible_subdivisions(source_model, source_eligible)
    model = _canonicalize_model_segments(model)
    current_model = model
    current_eligible = set(original_eligible)
    changed_steps = []
    seen = set()
    final_search = None
    last_composed = None
    last_source_composed = None
    bounded = False
    deadline_reached = False
    initial_mm = previous_mm = 0.0
    if pass_observer is not None:
        initial_state = _convergence_state(
            current_model, 0, 0.0, 0.0, "initial")
        initial_mm = initial_state["copper_mm"]
        initial_state["pass_gain_mm"] = 0.0
        initial_state["pass_gain_percent"] = 0.0
        initial_state["cumulative_gain_mm"] = 0.0
        initial_state["cumulative_gain_percent"] = 0.0
        previous_mm = initial_mm
        pass_observer(initial_state)

    for pass_index in range(max_passes + 1):
        if deadline is not None and time.monotonic() >= deadline:
            deadline_reached = True
            bounded = True
            final_search = GlossResult(warnings=[
                "Interactive planning time budget reached"])
            break
        signature = _model_geometry_signature(current_model)
        if signature in seen:
            if pass_observer is not None:
                pass_observer(_convergence_state(
                    current_model, len(changed_steps), initial_mm,
                    previous_mm, "cycle"))
            raise ValueError("Gloss convergence entered a geometry cycle")
        seen.add(signature)
        pass_kwargs = dict(kwargs)
        if batch_group_convergence and "converge_groups" not in pass_kwargs:
            # Preserve the established first-pass basin. Once that broad
            # global edit has opened local simplifications, converge each
            # independent net/layer group in its worker before reconciling the
            # next global state.
            pass_kwargs["converge_groups"] = pass_index > 0
        try:
            plans = generate_candidate_plans(
                current_model, current_eligible, **pass_kwargs)
        except PlanningDeadlineExceeded as error:
            deadline_reached = True
            bounded = True
            final_search = GlossResult(warnings=[str(error)])
            break
        changed_plans = [plan for plan in plans if plan.changed]
        if not changed_plans:
            final_search = plans[0]
            if pass_observer is not None:
                fixed_state = _convergence_state(
                    current_model, len(changed_steps), initial_mm,
                    previous_mm, "fixed_point")
                if final_search.warnings:
                    fixed_state["warnings"] = list(final_search.warnings)
                pass_observer(fixed_state)
            break
        if pass_index == max_passes:
            if pass_observer is not None:
                limited = _convergence_state(
                    current_model, len(changed_steps), initial_mm,
                    previous_mm, "max_passes")
                limited["next_plan_saved_mm"] = changed_plans[0].saved_mm
                pass_observer(limited)
            if not return_partial_on_limit:
                raise ValueError(
                    "Gloss did not reach a fixed point in {} changed passes".format(
                        max_passes))
            bounded = True
            final_search = plans[0]
            break
        accepted = None
        rejected_steps = []
        for step in changed_plans:
            proposed_model, proposed_eligible = _apply_to_model(
                current_model, current_eligible, step, pass_index)
            # The live adapter writes each collinear replacement as the
            # compact final track set. Continue searching on that same
            # topology; otherwise a final-only merge can expose an A1 -> A2
            # simplification after the engine has already claimed A1 fixed.
            proposed_model, proposed_eligible = _merge_final_collinear(
                proposed_model, set(proposed_eligible))
            proposed_model = _canonicalize_model_segments(proposed_model)
            if (_model_geometry_signature(proposed_model) ==
                    _model_geometry_signature(current_model)):
                rejected_steps.append(
                    "Candidate changes identities but not copper geometry")
                continue
            try:
                composed = _compose_refined_plan(
                    model, original_eligible, proposed_model,
                    proposed_eligible, changed_steps + [step], [])
                source_proposed, source_proposed_eligible = \
                    _restore_unchanged_subdivisions(
                        proposed_model, proposed_eligible, input_expansions)
                source_composed = _compose_refined_plan(
                    source_model, source_eligible, source_proposed,
                    source_proposed_eligible, changed_steps + [step], [])
            except ValueError as error:
                rejected_steps.append(str(error))
                continue
            accepted = (step, proposed_model, proposed_eligible, composed,
                        source_composed)
            break
        if accepted is None:
            final_search = GlossResult()
            final_search.warnings.append(
                "No cumulatively safe convergence step remained: " +
                "; ".join(sorted(set(rejected_steps))))
            break
        (step, proposed_model, proposed_eligible, last_composed,
         last_source_composed) = accepted
        changed_steps.append(step)
        if (deadline is not None and
                time.monotonic() >= deadline - 0.25):
            # The accepted proposed model and its composed one-shot plan have
            # already passed the complete engine gate. Do not spend additional
            # interactive time rebuilding the same topology solely to open a
            # later convergence pass.
            current_model = proposed_model
            current_eligible = proposed_eligible
            deadline_reached = True
            bounded = True
            final_search = GlossResult(warnings=[
                "Interactive planning time budget reached"])
            break
        # Normalize the next search state to the exact one-shot edit returned
        # to KiCad, rather than to the internal history of incremental edits.
        # This makes recursive openings in the compact route visible before
        # declaring an in-memory fixed point.
        current_model, current_eligible = _apply_to_model(
            model, original_eligible, last_composed, pass_index)
        current_model, current_eligible = _merge_final_collinear(
            current_model, set(current_eligible))
        current_model = _canonicalize_model_segments(current_model)
        if pass_observer is not None:
            state = _convergence_state(
                current_model, len(changed_steps), initial_mm, previous_mm,
                "changed", step)
            pass_observer(state)
            previous_mm = state["copper_mm"]
    else:  # pragma: no cover - the bounded loop always breaks or raises.
        raise ValueError("Gloss convergence ended unexpectedly")

    if not changed_steps:
        final_search.convergence_passes = 0
        final_search.fixed_point = not deadline_reached
        if subdivision_canonicalization_skipped:
            final_search.warnings.append(
                "Large interactive scope used bounded group planning; exhaustive "
                "segment-subdivision canonicalization is reserved for the CLI.")
        return final_search

    if deadline_reached and last_source_composed is not None:
        result = last_source_composed
    else:
        current_model, current_eligible = _restore_unchanged_subdivisions(
            current_model, current_eligible, input_expansions)
        result = _compose_refined_plan(
            source_model, source_eligible, current_model, current_eligible,
            changed_steps, [])
    # Nanometre quantization can collapse an analytical micro-adjustment back
    # to identity. Report applied passes, not discarded internal exploration.
    result.convergence_passes = len(changed_steps) if result.changed else 0
    result.fixed_point = not bounded
    if bounded:
        if deadline_reached:
            result.warnings.append(
                "Gloss stopped at the interactive planning time budget before "
                "reaching a fixed point.")
        else:
            result.warnings.append(
                "Gloss stopped at the {}-pass interactive guard before reaching "
                "a fixed point.".format(max_passes))
    if final_search is not None:
        result.chains_considered += final_search.chains_considered
        for key, value in final_search.search_counts.items():
            result.search_counts[key] = result.search_counts.get(key, 0) + value
        for key, value in final_search.blocking_nets.items():
            result.blocking_nets[key] = result.blocking_nets.get(key, 0) + value
        result.warnings.extend(final_search.warnings)
    if subdivision_canonicalization_skipped:
        result.warnings.append(
            "Large interactive scope used bounded group planning; exhaustive "
            "segment-subdivision canonicalization is reserved for the CLI.")
    return result
