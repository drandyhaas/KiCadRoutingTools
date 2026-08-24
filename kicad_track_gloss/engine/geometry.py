"""Dependency-free geometry kernels for track gloss."""

from __future__ import annotations

import math


EPS = 1e-9


def length(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    if den <= EPS:
        return length(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den))
    return length(p, (a[0] + t * dx, a[1] + t * dy))


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a, b, c, d):
    o1, o2, o3, o4 = _orient(a, b, c), _orient(a, b, d), _orient(c, d, a), _orient(c, d, b)
    if ((o1 > EPS and o2 < -EPS) or (o1 < -EPS and o2 > EPS)) and \
       ((o3 > EPS and o4 < -EPS) or (o3 < -EPS and o4 > EPS)):
        return True
    return (abs(o1) <= EPS and point_segment_distance(c, a, b) <= EPS or
            abs(o2) <= EPS and point_segment_distance(d, a, b) <= EPS or
            abs(o3) <= EPS and point_segment_distance(a, c, d) <= EPS or
            abs(o4) <= EPS and point_segment_distance(b, c, d) <= EPS)


def segment_distance(a, b, c, d):
    if segments_intersect(a, b, c, d):
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def octolinear_paths(a, b):
    """Yield shortest 0/45/90-degree paths between two fixed points."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    ax, ay = abs(dx), abs(dy)
    if ax <= EPS or ay <= EPS or abs(ax - ay) <= EPS:
        return [(a, b)]
    sx, sy = (1.0 if dx >= 0 else -1.0), (1.0 if dy >= 0 else -1.0)
    diagonal = min(ax, ay)
    if ax > ay:
        p1 = (a[0] + sx * diagonal, b[1])
        p2 = (b[0] - sx * diagonal, a[1])
    else:
        p1 = (b[0], a[1] + sy * diagonal)
        p2 = (a[0], b[1] - sy * diagonal)
    paths = []
    for p in (p1, p2):
        candidate = tuple(q for i, q in enumerate((a, p, b)) if i == 0 or length((a, p, b)[i - 1], q) > EPS)
        if candidate not in paths:
            paths.append(candidate)
    return paths


def point_in_polygon(point, polygon):
    inside = False
    x, y = point
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > y) != (yj > y):
            cross_x = (xj - xi) * (y - yi) / ((yj - yi) or EPS) + xi
            if x < cross_x:
                inside = not inside
        j = i
    return inside


def path_hits_polygon(a, b, polygon, margin=0.0):
    if point_in_polygon(a, polygon) or point_in_polygon(b, polygon):
        return True
    for c, d in zip(polygon, polygon[1:] + polygon[:1]):
        if segment_distance(a, b, c, d) < margin + EPS:
            return True
    return False

