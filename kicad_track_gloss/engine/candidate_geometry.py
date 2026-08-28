"""Candidate copper identity and local clearance validation.

The planner owns search order and convergence.  This module owns the exact,
API-neutral geometry gate used by every candidate and composed result.
"""

from __future__ import annotations

from collections import defaultdict
from functools import lru_cache

from .context import PlannerContext
from .geometry import (path_hits_polygon, point_segment_distance,
                       segment_distance)
from .model import segment_key
from .pads import segment_hits_pad


def effective_clearance(model, segment):
    if segment.clearance >= 0.0:
        return max(model.minimum_clearance, segment.clearance)
    return max(model.minimum_clearance,
               model.net_clearances.get(segment.net_id, 0.0))


def copper_signature(start, end, width, layer, net_id):
    a = (round(start[0], 6), round(start[1], 6))
    b = (round(end[0], 6), round(end[1], 6))
    return (min(a, b), max(a, b), round(width, 6), layer, net_id)


def segment_order_key(segment):
    """Geometry-first ordering; UUID is only an exact-duplicate tie-break."""
    return copper_signature(
        (segment.start_x, segment.start_y),
        (segment.end_x, segment.end_y), segment.width,
        segment.layer, segment.net_id) + (segment_key(segment),)


def retain_identity_replacements(model, result):
    """Keep original native items instead of removing and recreating them."""
    removed = set(result.remove_keys)
    originals = defaultdict(list)
    for segment in model.segments:
        key = segment_key(segment)
        if key in removed:
            originals[copper_signature(
                (segment.start_x, segment.start_y),
                (segment.end_x, segment.end_y), segment.width,
                segment.layer, segment.net_id)].append(key)

    cancelled = set()
    additions = []
    for addition in result.additions:
        signature = copper_signature(
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


@lru_cache(maxsize=131072)
def _capsule_interval(a, b, target_a, target_b, radius):
    """Parameter interval where ``a->b`` lies inside a target-track capsule."""
    if radius < -1e-12:
        return None
    radius = max(0.0, radius)

    def distance_at(t):
        point = (a[0] + t * (b[0] - a[0]),
                 a[1] + t * (b[1] - a[1]))
        return point_segment_distance(point, target_a, target_b)

    low, high = 0.0, 1.0
    for _ in range(64):
        left = (2.0 * low + high) / 3.0
        right = (low + 2.0 * high) / 3.0
        if distance_at(left) <= distance_at(right):
            high = right
        else:
            low = left
    minimum = (low + high) / 2.0
    if distance_at(minimum) > radius + 1e-9:
        return None

    if distance_at(0.0) <= radius + 1e-9:
        interval_start = 0.0
    else:
        outside, inside = 0.0, minimum
        for _ in range(64):
            middle = (outside + inside) / 2.0
            if distance_at(middle) <= radius + 1e-9:
                inside = middle
            else:
                outside = middle
        interval_start = inside

    if distance_at(1.0) <= radius + 1e-9:
        interval_end = 1.0
    else:
        inside, outside = minimum, 1.0
        for _ in range(64):
            middle = (inside + outside) / 2.0
            if distance_at(middle) <= radius + 1e-9:
                inside = middle
            else:
                outside = middle
        interval_end = inside
    return interval_start, interval_end


def _interval_is_covered(required, covers):
    if required is None:
        return True
    cursor = required[0]
    for start, end in sorted(covers):
        if end < cursor - 1e-9:
            continue
        if start > cursor + 1e-9:
            return False
        cursor = max(cursor, end)
        if cursor >= required[1] - 1e-9:
            return True
    return False


def _clearance_violation_is_preexisting(a, b, moving, foreign, required,
                                         immutable_cover_keys, context,
                                         coordinate_quantum_mm):
    """Whether all newly conflicting copper is inside immutable same-net copper."""
    violation = _capsule_interval(
        a, b, (foreign.start_x, foreign.start_y),
        (foreign.end_x, foreign.end_y),
        required - coordinate_quantum_mm)
    covers = []
    for existing in context.nearby_segments(a, b, 0.0, moving.width):
        if segment_key(existing) not in immutable_cover_keys:
            continue
        if existing.net_id != moving.net_id or existing.layer != moving.layer:
            continue
        if existing.width + 1e-9 < moving.width:
            continue
        cover_radius = (existing.width - moving.width) / 2.0
        interval = _capsule_interval(
            a, b, (existing.start_x, existing.start_y),
            (existing.end_x, existing.end_y), cover_radius)
        if interval is not None:
            covers.append(interval)
    return _interval_is_covered(violation, covers)


def path_blocker(model, path, moving, replaced_keys, clearance, context=None,
                 immutable_cover_keys=()):
    context = context or PlannerContext(model)
    moving_clearance = max(clearance, effective_clearance(model, moving))
    replaced_segments = [context.segment_by_key[key] for key in replaced_keys
                         if key in context.segment_by_key]

    def unchanged_copper(a, b):
        """Whether the candidate only retains part of removed copper."""
        return any(
            moving.width <= segment.width + model.coordinate_quantum_mm and
            point_segment_distance(a, (segment.start_x, segment.start_y),
                                   (segment.end_x, segment.end_y)) <= 1e-6 and
            point_segment_distance(b, (segment.start_x, segment.start_y),
                                   (segment.end_x, segment.end_y)) <= 1e-6
            for segment in replaced_segments)

    for a, b in zip(path, path[1:]):
        if unchanged_copper(a, b):
            continue
        edge_margin = model.copper_edge_clearance + moving.width / 2.0
        if model.board_outline:
            if not context.segment_inside_board(a, b, edge_margin):
                return "board_edge", 0
        elif model.board_bounds:
            x0, y0, x1, y1 = model.board_bounds
            if any(not (x0 + edge_margin <= p[0] <= x1 - edge_margin and
                        y0 + edge_margin <= p[1] <= y1 - edge_margin)
                   for p in (a, b)):
                return "board_edge", 0
        for other in context.nearby_segments(
                a, b, moving_clearance, moving.width):
            if segment_key(other) in replaced_keys:
                continue
            if other.layer != moving.layer or other.net_id == moving.net_id:
                continue
            pair_clearance = max(
                moving_clearance, effective_clearance(model, other))
            required = pair_clearance + (moving.width + other.width) / 2.0
            if segment_distance(a, b, (other.start_x, other.start_y),
                                (other.end_x, other.end_y)) < required - 1e-6:
                if _clearance_violation_is_preexisting(
                        a, b, moving, other, required,
                        immutable_cover_keys, context,
                        model.coordinate_quantum_mm):
                    continue
                return "foreign_track_clearance", other.net_id
        for obstacle in context.nearby_obstacles(
                a, b, moving_clearance, moving.width):
            if obstacle.net_id == moving.net_id:
                continue
            if obstacle.layers and moving.layer not in obstacle.layers:
                continue
            required = max(moving_clearance, obstacle.clearance) + \
                moving.width / 2.0 + obstacle.radius
            if point_segment_distance(
                    (obstacle.x, obstacle.y), a, b) < required - 1e-6:
                return obstacle.kind + "_clearance", obstacle.net_id
        for pad in context.nearby_pads(
                a, b, moving_clearance, moving.width):
            if pad.net_id == moving.net_id:
                continue
            if pad.layers and moving.layer not in pad.layers:
                continue
            pad_clearance = max(moving_clearance, pad.clearance)
            margin = max(0.0, pad_clearance + moving.width / 2.0 -
                         model.coordinate_quantum_mm)
            enclosing_radius = (pad.width * pad.width +
                                pad.height * pad.height) ** 0.5 / 2.0
            if point_segment_distance((pad.x, pad.y), a, b) >= \
                    enclosing_radius + margin:
                continue
            if segment_hits_pad(pad, a, b, margin=margin):
                return "pad_clearance", pad.net_id
        for keepout in context.nearby_keepouts(
                a, b, clearance, moving.width):
            if keepout.layers and moving.layer not in keepout.layers:
                continue
            if path_hits_polygon(
                    a, b, list(keepout.points),
                    clearance + moving.width / 2.0):
                return keepout.kind, 0
    return None
