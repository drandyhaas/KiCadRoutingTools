"""Detection and candidate positions for movable same-net T terminations."""

from __future__ import annotations

from collections import defaultdict

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


def movable_endpoint_pairs(start, end, start_targets, end_targets):
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
