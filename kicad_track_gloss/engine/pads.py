"""API-neutral pad-area geometry and deterministic boundary contacts."""

from __future__ import annotations

import math


def _local_coordinates(region, point):
    angle = math.radians(-region.orientation_degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    dx, dy = point[0] - region.x, point[1] - region.y
    return dx * cosine - dy * sine, dx * sine + dy * cosine


def pad_contains(region, point, tolerance=1e-9):
    """Return whether a point lies in a supported pad copper shape."""
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


def pad_contact_points(reference, original, regions):
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
                for _iteration in range(50):
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
