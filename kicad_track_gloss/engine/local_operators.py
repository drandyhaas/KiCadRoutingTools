"""Exact local gloss operators which preserve the routed corridor."""

from __future__ import annotations

from dataclasses import replace
import math

from .candidate_geometry import path_blocker
from .geometry import length


def maximal_corner_chamfer_path(
        points, i, span, model, context, clearance, replaced_keys,
        immutable_cover_keys, check_deadline, is_octolinear, deadline=None,
        cancel_check=None):
    """Cut both sides of one 90-degree corner to the largest safe 45-degree chord."""
    if len(span) != 2:
        return None
    first, second = span
    tolerance = model.coordinate_quantum_mm
    if (abs(first.width - second.width) > tolerance or
            abs(first.clearance - second.clearance) > tolerance):
        return None
    a, corner, c = points[i], points[i + 1], points[i + 2]
    first_length = length(a, corner)
    second_length = length(corner, c)
    if first_length <= tolerance or second_length <= tolerance:
        return None
    incoming = ((corner[0] - a[0]) / first_length,
                (corner[1] - a[1]) / first_length)
    outgoing = ((c[0] - corner[0]) / second_length,
                (c[1] - corner[1]) / second_length)
    if abs(incoming[0] * outgoing[0] +
           incoming[1] * outgoing[1]) > tolerance:
        return None
    if (not is_octolinear(a, corner, tolerance) or
            not is_octolinear(corner, c, tolerance)):
        return None

    moving = replace(first, width=first.width)

    def cut_points(distance):
        before = (corner[0] - incoming[0] * distance,
                  corner[1] - incoming[1] * distance)
        after = (corner[0] + outgoing[0] * distance,
                 corner[1] + outgoing[1] * distance)
        return before, after

    def blocked(distance):
        check_deadline(deadline, cancel_check)
        before, after = cut_points(distance)
        return path_blocker(
            model, (before, after), moving, replaced_keys, clearance,
            context, immutable_cover_keys)

    maximum = min(first_length, second_length)
    if blocked(maximum) is None:
        safe = maximum
    else:
        safe, unsafe = 0.0, maximum
        while unsafe - safe > tolerance:
            middle = (safe + unsafe) / 2.0
            if blocked(middle) is None:
                safe = middle
            else:
                unsafe = middle
        safe = math.floor(safe / tolerance) * tolerance
        while safe > tolerance and blocked(safe) is not None:
            safe -= tolerance
    if safe <= tolerance:
        return None
    before, after = cut_points(safe)
    raw = (a, before, after, c)
    path = tuple(point for index, point in enumerate(raw)
                 if index == 0 or length(raw[index - 1], point) > tolerance)
    return path if len(path) >= 2 else None


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _line_intersection(point_a, direction_a, point_b, direction_b):
    denominator = _cross(direction_a, direction_b)
    if abs(denominator) <= 1e-12:
        return None
    delta = (point_b[0] - point_a[0], point_b[1] - point_a[1])
    scale = _cross(delta, direction_b) / denominator
    return (point_a[0] + scale * direction_a[0],
            point_a[1] + scale * direction_a[1])


def internal_segment_translation_paths(
        points, i, span, model, context, clearance, replaced_keys,
        immutable_cover_keys, check_deadline, deadline=None,
        cancel_check=None):
    """Slide the middle of three segments to its exact useful safe limits."""
    if len(span) != 3:
        return ()
    tolerance = model.coordinate_quantum_mm
    if len({(round(item.width, 6), round(item.clearance, 6))
            for item in span}) != 1:
        return ()
    a, b, c, d = points[i:i + 4]
    directions = (
        (b[0] - a[0], b[1] - a[1]),
        (c[0] - b[0], c[1] - b[1]),
        (d[0] - c[0], d[1] - c[1]),
    )
    lengths = [math.hypot(*direction) for direction in directions]
    if min(lengths) <= tolerance:
        return ()
    first = (directions[0][0] / lengths[0],
             directions[0][1] / lengths[0])
    middle = (directions[1][0] / lengths[1],
              directions[1][1] / lengths[1])
    last = (directions[2][0] / lengths[2],
            directions[2][1] / lengths[2])
    if (abs(_cross(first, middle)) <= 1e-12 or
            abs(_cross(last, middle)) <= 1e-12):
        return ()
    normal = (-middle[1], middle[0])

    def raw_path(offset):
        shifted = (b[0] + offset * normal[0],
                   b[1] + offset * normal[1])
        before = _line_intersection(a, first, shifted, middle)
        after = _line_intersection(d, last, shifted, middle)
        if before is None or after is None:
            return None
        return a, before, after, d

    def measures(offset):
        raw = raw_path(offset)
        if raw is None:
            return None
        _a, before, after, _d = raw
        return (
            (before[0] - a[0]) * first[0] +
            (before[1] - a[1]) * first[1],
            (d[0] - after[0]) * last[0] +
            (d[1] - after[1]) * last[1],
            (after[0] - before[0]) * middle[0] +
            (after[1] - before[1]) * middle[1],
        )

    base = measures(0.0)
    shifted = measures(1.0)
    if base is None or shifted is None or min(base) < -tolerance:
        return ()
    lower, upper = -math.inf, math.inf
    for initial, at_one in zip(base, shifted):
        slope = at_one - initial
        if abs(slope) <= 1e-12:
            if initial < -tolerance:
                return ()
            continue
        root = -initial / slope
        if slope > 0.0:
            lower = max(lower, root)
        else:
            upper = min(upper, root)
    targets = [value for value in (lower, upper)
               if math.isfinite(value) and abs(value) > tolerance]
    moving = span[0]

    def normalized_path(offset):
        raw = raw_path(offset)
        if raw is None:
            return None
        path = tuple(point for index, point in enumerate(raw)
                     if index == 0 or
                     length(raw[index - 1], point) > tolerance)
        return path if len(path) >= 2 else None

    def blocked(offset):
        check_deadline(deadline, cancel_check)
        path = normalized_path(offset)
        if path is None:
            return True
        return path_blocker(
            model, path, moving, replaced_keys, clearance, context,
            immutable_cover_keys) is not None

    old_length = sum(length(x, y) for x, y in zip((a, b, c), (b, c, d)))
    candidates = []
    for target in targets:
        check_deadline(deadline, cancel_check)
        target_path = normalized_path(target)
        if target_path is None:
            continue
        target_length = sum(length(x, y)
                            for x, y in zip(target_path, target_path[1:]))
        if target_length >= old_length - tolerance:
            continue
        if blocked(target):
            safe, unsafe = 0.0, target
            while abs(unsafe - safe) > tolerance:
                middle_offset = (safe + unsafe) / 2.0
                if blocked(middle_offset):
                    unsafe = middle_offset
                else:
                    safe = middle_offset
            target_path = normalized_path(safe)
        if target_path is None:
            continue
        new_length = sum(length(x, y)
                         for x, y in zip(target_path, target_path[1:]))
        if new_length < old_length - tolerance and target_path not in candidates:
            candidates.append(target_path)
    return tuple(candidates)


__all__ = ("internal_segment_translation_paths",
           "maximal_corner_chamfer_path")
