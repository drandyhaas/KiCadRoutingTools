"""KiCad's integer segment geometry, ported for measurements that must agree
with KiCad's DRC to the nanometre (#1303).

KiCad stores coordinates as integer nanometres (IU) and its diff-pair coupling
test (pcbnew/drc/drc_test_provider_diff_pair_coupling.cpp) decides "parallel",
"overlapping" and "how far apart" in that integer arithmetic. A float port
disagrees with it on exactly the cases that matter: two diagonal tracks whose
written endpoints are 1 nm off parallel are NOT parallel to KiCad's
`SEG::Intersect(..., aLines=true)` -- unless their lines meet beyond the int32
coordinate range, in which case KiCad reports no intersection and treats them
as parallel after all. So this module mirrors libs/kimath (KiCad 10.0.0):
`SEG::intersects`, `LineProject`, `TCoef`, `NearestPoint(s)`, `rescale` (the
__int128 path: round half away from zero, truncating division) and
`VECTOR2::EuclideanNorm` (exact for axis and 45-degree vectors, KiROUND of
hypot otherwise). Python ints are unbounded, so every product is exact.

Points are (x, y) tuples of ints in IU; `iu(mm)` converts the way KiCad's
board parser does (KiROUND of mm * 1e6).
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

Pt = Tuple[int, int]

#: int32 coordinate range (VECTOR2I::coord_type): an intersection outside it
#: is "no intersection" to SEG::intersects.
_COORD_MAX = 2 ** 31 - 1
_COORD_MIN = -(2 ** 31)


def kiround(v: float) -> int:
    """KiROUND: round half away from zero."""
    return int(math.floor(v + 0.5)) if v >= 0 else -int(math.floor(-v + 0.5))


def iu(mm: float) -> int:
    """mm -> KiCad internal units (nm), as the board parser converts."""
    return kiround(mm * 1e6)


def _tdiv(a: int, b: int) -> int:
    """C++ integer division: truncates toward zero."""
    q = abs(a) // abs(b)
    return -q if (a < 0) != (b < 0) else q


def rescale(numerator: int, value: int, denominator: int) -> int:
    """KiCad's int64 rescale (numerator * value / denominator), the
    __int128 branch: rounds half away from zero."""
    num = numerator * value
    half = _tdiv(denominator, 2)
    if (num < 0) != (denominator < 0):
        return _tdiv(num - half, denominator)
    return _tdiv(num + half, denominator)


def cross(ax: int, ay: int, bx: int, by: int) -> int:
    return ax * by - ay * bx


def dot(ax: int, ay: int, bx: int, by: int) -> int:
    return ax * bx + ay * by


def norm(dx: int, dy: int) -> int:
    """VECTOR2I::EuclideanNorm: exact on the axes and at 45 degrees."""
    if abs(dx) == abs(dy):
        return kiround(abs(dx) * math.sqrt(2))
    if dx == 0:
        return abs(dy)
    if dy == 0:
        return abs(dx)
    return kiround(math.hypot(dx, dy))


def sq_len(a: Pt, b: Pt) -> int:
    dx, dy = b[0] - a[0], b[1] - a[1]
    return dx * dx + dy * dy


def tcoef(a: Pt, b: Pt, p: Pt) -> int:
    """SEG::TCoef: (B - A) . (P - A)."""
    return dot(b[0] - a[0], b[1] - a[1], p[0] - a[0], p[1] - a[1])


def line_project(a: Pt, b: Pt, p: Pt) -> Pt:
    """SEG::LineProject: P projected on the infinite line AB."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    if l2 == 0:
        return a
    t = dot(dx, dy, p[0] - a[0], p[1] - a[1])
    return (a[0] + rescale(t, dx, l2), a[1] + rescale(t, dy, l2))


def nearest_point(a: Pt, b: Pt, p: Pt) -> Pt:
    """SEG::NearestPoint: the point of segment AB nearest P."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    if l2 == 0:
        return a
    t = dot(dx, dy, p[0] - a[0], p[1] - a[1])
    if t < 0:
        return a
    if t > l2:
        return b
    return (a[0] + rescale(t, dx, l2), a[1] + rescale(t, dy, l2))


def _collinear_overlap(a: Pt, b: Pt, c: Pt, d: Pt, use_x: bool) -> bool:
    """SEG::checkCollinearOverlap without the ignore-endpoints case: do two
    collinear segments share at least one point?"""
    i = 0 if use_x else 1
    s1, e1 = sorted((a[i], b[i]))
    s2, e2 = sorted((c[i], d[i]))
    return max(s1, s2) <= min(e1, e2)


def intersection(a: Pt, b: Pt, c: Pt, d: Pt, lines: bool = False) -> Optional[Pt]:
    """SEG(a, b).Intersect(SEG(c, d), aIgnoreEndpoints=false, aLines), or
    None. With `lines`, the two are infinite lines: parallel distinct lines
    never meet, collinear ones always do, and -- KiCad's own rule -- lines
    whose meeting point falls outside the int32 coordinate range do NOT
    intersect, so two lines a few nm off parallel count as parallel."""
    d1 = (b[0] - a[0], b[1] - a[1])
    d2 = (d[0] - c[0], d[1] - c[1])
    off = (c[0] - a[0], c[1] - a[1])
    det = cross(d2[0], d2[1], d1[0], d1[1])
    if det == 0:
        if cross(d1[0], d1[1], off[0], off[1]) != 0:
            return None                      # parallel, not collinear
        if lines:
            return ((a[0] + c[0]) // 2, (a[1] + c[1]) // 2)
        if _collinear_overlap(a, b, c, d, abs(d1[0]) >= abs(d1[1])):
            return c                         # a shared point; only "some" is used
        return None
    p1 = cross(d1[0], d1[1], off[0], off[1])
    p2 = cross(d2[0], d2[1], off[0], off[1])
    if not lines:
        if det > 0:
            if p1 < 0 or p1 > det or p2 < 0 or p2 > det:
                return None
        else:
            if p1 > 0 or p1 < det or p2 > 0 or p2 < det:
                return None
    rx = c[0] + rescale(p1, d2[0], det)
    ry = c[1] + rescale(p1, d2[1], det)
    if not (_COORD_MIN <= rx <= _COORD_MAX and _COORD_MIN <= ry <= _COORD_MAX):
        return None
    return (rx, ry)


def intersects(a: Pt, b: Pt, c: Pt, d: Pt, lines: bool = False) -> bool:
    """`intersection(...) is not None`."""
    return intersection(a, b, c, d, lines) is not None


def nearest_points(a: Pt, b: Pt, c: Pt, d: Pt) -> Tuple[Pt, Pt, int]:
    """SEG(a, b).NearestPoints(SEG(c, d)): (on AB, on CD, squared distance).
    Intersecting segments return their crossing at distance 0."""
    x = intersection(a, b, c, d)
    if x is not None:
        return x, x, 0
    po = (nearest_point(c, d, a), nearest_point(c, d, b),
          nearest_point(a, b, c), nearest_point(a, b, d))
    outs = ((a, po[0]), (b, po[1]), (po[2], c), (po[3], d))
    dists = (sq_len(po[0], a), sq_len(po[1], b), sq_len(po[2], c), sq_len(po[3], d))
    i = min(range(4), key=lambda k: (dists[k], k))
    return outs[i][0], outs[i][1], dists[i]


def common_parallel_projection(pa: Pt, pb: Pt, na: Pt, nb: Pt
                               ) -> Optional[Tuple[Pt, Pt, Pt, Pt]]:
    """KiCad's commonParallelProjection(p, n): the parts of P and N that face
    each other, as (pClip.A, pClip.B, nClip.A, nClip.B), or None when their
    projections on P's line do not overlap."""
    proj_a = line_project(pa, pb, na)
    proj_b = line_project(pa, pb, nb)
    t_a, t_b = 0, tcoef(pa, pb, pb)
    tp_a, tp_b = tcoef(pa, pb, proj_a), tcoef(pa, pb, proj_b)
    if t_b < t_a:
        t_a, t_b = t_b, t_a
    if tp_b < tp_a:
        tp_a, tp_b = tp_b, tp_a
    if t_b <= tp_a or t_a >= tp_b:
        return None
    tv = sorted((0, tcoef(pa, pb, pb), tcoef(pa, pb, proj_a), tcoef(pa, pb, proj_b)))
    l2 = sq_len(pa, pb)
    dx, dy = pb[0] - pa[0], pb[1] - pa[1]
    ca = (pa[0] + rescale(dx, tv[1], l2), pa[1] + rescale(dy, tv[1], l2))
    cb = (pa[0] + rescale(dx, tv[2], l2), pa[1] + rescale(dy, tv[2], l2))
    return ca, cb, line_project(na, nb, ca), line_project(na, nb, cb)
