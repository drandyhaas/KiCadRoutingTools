#!/usr/bin/env python3
"""Retract and grow: the one channel a movie has that a still does not (#946
item 13, #1022).

A rip is two frames of red at `rip_hold` and then the copper is gone -- which
is the least legible way to show the single event the movie exists to explain.
A still frame can only use colour and a mark; a MOVIE has **time**, and
direction survives every colour deficiency there is. Copper retracting toward
its anchor and copper growing outward from it are opposite motions, readable
with no colour at all.

This is not an alternative to the dash (#1013). The dash is the floor: cheap,
immediate, and it makes a *still* frame legible. Motion is what the medium is
actually for, and it ships on top.

**PURE GEOMETRY, NO PIL, NO RENDERER.** It takes trace-style rows -- the
`[sx, sy, ex, ey, width, layer_index]` that `route_trace.seg_key_row` keys --
and returns rows, so the whole animation can be asserted as data before
anything is drawn. Same posture as `frame_layout` and `movie_camera.plan_shots`.

**IT RETRACTS FROM THE FAR END, NOT EVERYWHERE AT ONCE.** Shrinking every
doomed segment toward its own midpoint by the same fraction is much easier and
reads as the copper dissolving, which is not what happened. A rip pulls a track
back from its frontier toward the copper that stays, so the segments are
ORDERED by distance from an anchor and consumed from the far end, with the
frontier segment drawn partially. The anchor is the doomed set's endpoint
nearest the copper that survives -- because that is the end a track is pulled
back TO.

**THE STAGE COUNT IS CONSTANT AND THE GEOMETRY IS BOUNDED.** Every stage
returns rows whose endpoints lie on the original segments, so nothing can be
drawn outside the board, and a caller gets exactly `stages` lists whatever the
copper looks like. Frame COUNT changes; frame SIZE cannot.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

#: How many stages a retraction or a growth is drawn over. Four reads as motion
#: at 6 fps (the movie default) without making a rip-heavy film four times
#: longer than it was: only rips and RESTORES move, and both are rare compared
#: with plain adds.
MOTION_STAGES = 4


def _len(row) -> float:
    return math.hypot(row[2] - row[0], row[3] - row[1])


def _dist(ax, ay, bx, by) -> float:
    return math.hypot(bx - ax, by - ay)


def anchor_for(rows: Sequence[Sequence], live: Sequence[Sequence] = ()
               ) -> Optional[Tuple[float, float]]:
    """The point the doomed (or arriving) copper is pulled back to / grows out
    of: the endpoint of `rows` closest to any endpoint of `live`.

    `live` is the copper that is NOT changing. With none -- a whole net being
    ripped, or the first copper on the board -- the anchor is the endpoint
    closest to the set's own centroid, which keeps the motion converging on one
    place instead of on an arbitrary end.
    """
    pts = []
    for r in rows:
        pts.append((r[0], r[1]))
        pts.append((r[2], r[3]))
    if not pts:
        return None
    if live:
        best, bd = pts[0], None
        for px, py in pts:
            for q in live:
                d = min(_dist(px, py, q[0], q[1]), _dist(px, py, q[2], q[3]))
                if bd is None or d < bd:
                    bd, best = d, (px, py)
        return best
    cx = sum(p[0] for p in pts) / float(len(pts))
    cy = sum(p[1] for p in pts) / float(len(pts))
    return min(pts, key=lambda p: _dist(p[0], p[1], cx, cy))


def order_from(rows: Sequence[Sequence], anchor) -> List[list]:
    """`rows`, nearest-first from `anchor`, each ORIENTED so its near end is
    its start.

    Orientation is what makes a partial segment mean something: a fraction of
    an oriented segment is the part still attached to the anchor, and a
    fraction of an unoriented one is a stub floating wherever the file happened
    to spell the endpoints. `route_trace` rows carry endpoint order as written,
    and `awx/evolve_movie.self_test` already pins that "endpoint order is not a
    change" for the copper diff -- so nothing downstream depends on the order
    this function flips.
    """
    if anchor is None:
        return [list(r) for r in rows]
    ax, ay = anchor
    out = []
    for r in rows:
        r = list(r)
        if _dist(ax, ay, r[2], r[3]) < _dist(ax, ay, r[0], r[1]):
            r[0], r[1], r[2], r[3] = r[2], r[3], r[0], r[1]
        out.append(r)
    out.sort(key=lambda r: (_dist(ax, ay, r[0], r[1]),
                            _dist(ax, ay, r[2], r[3])))
    return out


def _prefix(ordered: Sequence[Sequence], keep: float) -> List[list]:
    """The first `keep` mm of `ordered`, measured along the chain from the
    anchor. The frontier segment is cut, not dropped."""
    out = []
    left = keep
    for r in ordered:
        L = _len(r)
        if left <= 1e-9:
            break
        if L <= left or L <= 1e-9:
            out.append(list(r))
            left -= L
            continue
        f = max(0.0, min(1.0, left / L))
        out.append([r[0], r[1],
                    r[0] + (r[2] - r[0]) * f,
                    r[1] + (r[3] - r[1]) * f,
                    r[4], r[5]])
        left = 0.0
        break
    return out


def stages(rows: Sequence[Sequence], *, anchor=None, live=(),
           n: int = MOTION_STAGES, grow: bool = False) -> List[List[list]]:
    """`n` lists of rows: the copper retracting toward the anchor, or growing
    out of it.

    Retracting, stage `i` holds `(n - 1 - i) / (n - 1)` of the total length, so
    the LAST stage is empty -- the copper is gone, which is the event. Growing,
    stage `i` holds `(i + 1) / n`, so the last stage is the whole thing and the
    caller's live state and the last frame agree.

    Returns `[]` for an empty input and for `n < 2`, which is the documented
    degradation: `--rip-hold 0` asks for the old cut, and a one-stage motion is
    not a motion.
    """
    rows = [list(r) for r in rows if r is not None]
    if not rows or n < 2:
        return []
    a = anchor if anchor is not None else anchor_for(rows, live)
    ordered = order_from(rows, a)
    total = sum(_len(r) for r in ordered)
    if total <= 1e-9:
        # Zero-length copper (a stitch of coincident points) has no direction
        # to move in. Returning [] is honest; inventing one is not.
        return []
    out = []
    for i in range(n):
        f = ((i + 1) / float(n)) if grow else (1.0 - (i + 1) / float(n))
        out.append(_prefix(ordered, total * max(0.0, min(1.0, f))))
    return out
