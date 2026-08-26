"""API-neutral pad-area geometry and deterministic boundary contacts."""

from __future__ import annotations

import math
from functools import lru_cache

from .geometry import (EPS, path_hits_polygon, point_in_polygon,
                       point_segment_distance, segment_distance)


@lru_cache(maxsize=1024)
def _rotation(orientation_degrees):
    angle = math.radians(-orientation_degrees)
    return math.cos(angle), math.sin(angle)


def _local_coordinates(region, point):
    cosine, sine = _rotation(region.orientation_degrees)
    dx, dy = point[0] - region.x, point[1] - region.y
    return dx * cosine - dy * sine, dx * sine + dy * cosine


def pad_contains(region, point, tolerance=1e-9):
    """Return whether a point lies in a supported pad copper shape."""
    if region.shape == "custom":
        return any(
            point_in_polygon(point, outer) and
            not any(point_in_polygon(point, hole) for hole in holes)
            for outer, holes in region.polygons)
    x, y = _local_coordinates(region, point)
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
    radius = min(max(region.corner_radius, 0.0), half_width, half_height)
    if region.shape == "rect" or radius <= tolerance:
        return ax <= half_width + tolerance and ay <= half_height + tolerance
    qx = max(ax - (half_width - radius), 0.0)
    qy = max(ay - (half_height - radius), 0.0)
    return qx * qx + qy * qy <= (radius + tolerance) ** 2


def _vertex(point):
    return round(point[0], 6), round(point[1], 6)


@lru_cache(maxsize=16384)
def _pad_contact_points_cached(reference, original, regions):
    """Return bounded 0/45/90 boundary contacts nearest ``reference``."""
    contacts = {_vertex(original)}
    directions = ((1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, -1.0))
    for region in regions:
        for dx, dy in directions:
            denominator = dx * dx + dy * dy
            center_t = ((region.x - reference[0]) * dx +
                        (region.y - reference[1]) * dy) / denominator
            center_point = (reference[0] + center_t * dx,
                            reference[1] + center_t * dy)
            if not pad_contains(region, center_point):
                continue
            extent = math.hypot(region.width, region.height) + 1.0
            boundaries = []
            for sign in (-1.0, 1.0):
                inside_t = center_t
                outside_t = center_t + sign * extent
                # Contacts are stored at micrometre precision. Thirty-two
                # bisections already resolve a board-sized pad far below one
                # nanometre, so further iterations only repeat identical work.
                for _iteration in range(32):
                    middle = (inside_t + outside_t) / 2.0
                    point = (reference[0] + middle * dx,
                             reference[1] + middle * dy)
                    if pad_contains(region, point):
                        inside_t = middle
                    else:
                        outside_t = middle
                boundaries.append(_vertex((reference[0] + inside_t * dx,
                                           reference[1] + inside_t * dy)))
            contacts.add(min(boundaries, key=lambda point:
                             (point[0] - reference[0]) ** 2 +
                             (point[1] - reference[1]) ** 2))
    return sorted(contacts)


def pad_contact_points(reference, original, regions):
    return _pad_contact_points_cached(
        tuple(reference), tuple(original), tuple(regions))


def _rectangle(half_width, half_height):
    return [(-half_width, -half_height), (half_width, -half_height),
            (half_width, half_height), (-half_width, half_height)]


def _segment_hits_rectangle(a, b, half_width, half_height, margin):
    if half_width <= 0.0 or half_height <= 0.0:
        return False
    return path_hits_polygon(
        a, b, _rectangle(half_width, half_height), margin)


def segment_hits_pad(region, a, b, margin=0.0):
    """Test a track centreline against the real supported pad copper shape."""
    if region.shape == "custom":
        for outer, holes in region.polygons:
            if not path_hits_polygon(a, b, list(outer), margin):
                continue
            # Copper is ``outer - holes``.  A segment is clear only when it is
            # wholly inside one hole and remains at least ``margin`` from that
            # hole's copper boundary.  Treat ambiguous cases as copper.
            safely_inside_hole = False
            for hole in holes:
                if (point_in_polygon(a, hole) and point_in_polygon(b, hole) and
                        all(segment_distance(a, b, c, d) >= margin + EPS
                            for c, d in zip(hole, hole[1:] + hole[:1]))):
                    safely_inside_hole = True
                    break
            if not safely_inside_hole:
                return True
        return False
    a = _local_coordinates(region, a)
    b = _local_coordinates(region, b)
    half_width, half_height = region.width / 2.0, region.height / 2.0
    if region.shape == "circle":
        return point_segment_distance((0.0, 0.0), a, b) < \
            min(half_width, half_height) + margin - 1e-9
    if region.shape == "oval":
        if half_width >= half_height:
            core = half_width - half_height
            c, d, radius = (-core, 0.0), (core, 0.0), half_height
        else:
            core = half_height - half_width
            c, d, radius = (0.0, -core), (0.0, core), half_width
        return segment_distance(a, b, c, d) < radius + margin - 1e-9
    radius = min(max(region.corner_radius, 0.0), half_width, half_height)
    if region.shape == "rect" or radius <= 1e-9:
        return _segment_hits_rectangle(
            a, b, half_width, half_height, margin)

    # A rounded rectangle is the union of two central rectangles and four
    # corner circles. Testing each primitive with the requested margin gives
    # the exact Minkowski clearance without a circumscribed-circle false hit.
    if _segment_hits_rectangle(a, b, half_width - radius,
                               half_height, margin):
        return True
    if _segment_hits_rectangle(a, b, half_width,
                               half_height - radius, margin):
        return True
    for x in (-half_width + radius, half_width - radius):
        for y in (-half_height + radius, half_height - radius):
            if point_segment_distance((x, y), a, b) < radius + margin - 1e-9:
                return True
    return False
