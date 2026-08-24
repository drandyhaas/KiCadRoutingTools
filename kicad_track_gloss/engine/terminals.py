"""Detection and candidate positions for movable same-net T terminations."""

from __future__ import annotations

from collections import defaultdict
import math

from .geometry import point_segment_distance
from .model import segment_key


def vertex(x, y):
    return round(x, 6), round(y, 6)


def find_track_terminal_targets(model, eligible_segment_keys, tolerance=1e-5):
    """Find eligible vertices electrically terminating on immutable tracks."""
    eligible = {str(key) for key in eligible_segment_keys}
    immutable = [segment for segment in model.segments
                 if segment_key(segment) not in eligible]
    targets = defaultdict(dict)
    for segment in model.segments:
        if segment_key(segment) not in eligible:
            continue
        for point in ((segment.start_x, segment.start_y),
                      (segment.end_x, segment.end_y)):
            terminal = (segment.net_id, segment.layer, vertex(*point))
            for other in immutable:
                if (other.net_id != segment.net_id or other.layer != segment.layer or
                        other.arc):
                    continue
                if point_segment_distance(
                        point, (other.start_x, other.start_y),
                        (other.end_x, other.end_y)) <= tolerance:
                    targets[terminal][segment_key(other)] = other
    return {terminal: tuple(found[key] for key in sorted(found))
            for terminal, found in targets.items()}


def find_track_terminal_vertices(model, eligible_segment_keys, tolerance=1e-5):
    return set(find_track_terminal_targets(
        model, eligible_segment_keys, tolerance))


def _pad_local(region, point):
    angle = math.radians(-region.orientation_degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    dx, dy = point[0] - region.x, point[1] - region.y
    return dx * cosine - dy * sine, dx * sine + dy * cosine


def _pad_contains(region, point, tolerance=1e-9):
    x, y = _pad_local(region, point)
    half_width, half_height = region.width / 2.0, region.height / 2.0
    ax, ay = abs(x), abs(y)
    if region.shape == "circle":
        radius = min(half_width, half_height)
        return x * x + y * y <= (radius + tolerance) ** 2
    if region.shape == "oval":
        if half_width >= half_height:
            core, radius = half_width - half_height, half_height
            nearest = (max(-core, min(core, x)), 0.0)
        else:
            core, radius = half_height - half_width, half_width
            nearest = (0.0, max(-core, min(core, y)))
        return ((x - nearest[0]) ** 2 + (y - nearest[1]) ** 2 <=
                (radius + tolerance) ** 2)
    if region.shape == "fallback":
        if half_width <= 0 or half_height <= 0:
            return False
        return ((x / half_width) ** 2 + (y / half_height) ** 2 <=
                1.0 + tolerance)
    radius = min(max(region.corner_radius, 0.0), half_width, half_height)
    if region.shape == "rect" or radius <= tolerance:
        return ax <= half_width + tolerance and ay <= half_height + tolerance
    qx = max(ax - (half_width - radius), 0.0)
    qy = max(ay - (half_height - radius), 0.0)
    return qx * qx + qy * qy <= (radius + tolerance) ** 2


def find_pad_terminal_targets(model, eligible_segment_keys):
    """Find same-net pad copper regions containing eligible endpoints."""
    eligible = {str(key) for key in eligible_segment_keys}
    targets = defaultdict(list)
    for segment in model.segments:
        if segment_key(segment) not in eligible:
            continue
        for point in ((segment.start_x, segment.start_y),
                      (segment.end_x, segment.end_y)):
            terminal = (segment.net_id, segment.layer, vertex(*point))
            for region in model.pad_regions:
                if (region.net_id == segment.net_id and
                        (not region.layers or segment.layer in region.layers) and
                        _pad_contains(region, point, tolerance=1e-6)):
                    targets[terminal].append(region)
    return {terminal: tuple(regions) for terminal, regions in targets.items()}


def _project_to_segment(point, segment):
    a = (segment.start_x, segment.start_y)
    b = (segment.end_x, segment.end_y)
    dx, dy = b[0] - a[0], b[1] - a[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1e-18:
        return a
    t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / denominator
    t = max(0.0, min(1.0, t))
    return vertex(a[0] + t * dx, a[1] + t * dy)


def _sliding_contact_points(reference, original, targets):
    contacts = {vertex(*original)}
    directions = ((1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, -1.0))
    for target in targets:
        a = (target.start_x, target.start_y)
        b = (target.end_x, target.end_y)
        contacts.update((vertex(*a), vertex(*b),
                         _project_to_segment(reference, target)))
        wx, wy = b[0] - a[0], b[1] - a[1]
        for vx, vy in directions:
            denominator = vx * wy - vy * wx
            if abs(denominator) <= 1e-12:
                continue
            cx, cy = a[0] - reference[0], a[1] - reference[1]
            t = (cx * vy - cy * vx) / denominator
            if -1e-9 <= t <= 1.0 + 1e-9:
                contacts.add(vertex(a[0] + t * wx, a[1] + t * wy))
    return sorted(contacts)


def _pad_contact_points(reference, original, regions):
    contacts = {vertex(*original)}
    directions = ((1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, -1.0))
    for region in regions:
        for dx, dy in directions:
            denominator = dx * dx + dy * dy
            center_t = ((region.x - reference[0]) * dx +
                        (region.y - reference[1]) * dy) / denominator
            center_point = (reference[0] + center_t * dx,
                            reference[1] + center_t * dy)
            if not _pad_contains(region, center_point):
                continue
            extent = math.hypot(region.width, region.height) + 1.0
            boundaries = []
            for sign in (-1.0, 1.0):
                inside_t = center_t
                outside_t = center_t + sign * extent
                for _iteration in range(50):
                    middle = (inside_t + outside_t) / 2.0
                    point = (reference[0] + middle * dx,
                             reference[1] + middle * dy)
                    if _pad_contains(region, point):
                        inside_t = middle
                    else:
                        outside_t = middle
                boundaries.append(vertex(reference[0] + inside_t * dx,
                                         reference[1] + inside_t * dy))
            # The farther intersection and interior projection cannot shorten
            # a path from ``reference`` more than the nearest pad boundary.
            contacts.add(min(boundaries, key=lambda point:
                             (point[0] - reference[0]) ** 2 +
                             (point[1] - reference[1]) ** 2))
    return sorted(contacts)


def _terminal_points(reference, original, track_targets, pad_targets):
    points = {vertex(*original)}
    if track_targets:
        points.update(_sliding_contact_points(reference, original, track_targets))
    if pad_targets:
        points.update(_pad_contact_points(reference, original, pad_targets))
    return sorted(points)


def movable_endpoint_pairs(start, end, start_targets, end_targets,
                           start_pads=(), end_pads=()):
    starts = _terminal_points(end, start, start_targets, start_pads)
    ends = _terminal_points(start, end, end_targets, end_pads)
    pairs = {(candidate_start, candidate_end)
             for candidate_start in starts for candidate_end in ends}
    if (start_targets or start_pads) and (end_targets or end_pads):
        # Refine one endpoint against each initial point on the other side,
        # but keep the pairs coupled. A Cartesian product of every refinement
        # is quadratic and mostly combines geometrically unrelated contacts.
        for candidate_end in ends:
            pairs.update((candidate_start, candidate_end)
                         for candidate_start in _terminal_points(
                             candidate_end, start, start_targets, start_pads))
        for candidate_start in starts:
            pairs.update((candidate_start, candidate_end)
                         for candidate_end in _terminal_points(
                             candidate_start, end, end_targets, end_pads))
    return iter(sorted(pairs))
