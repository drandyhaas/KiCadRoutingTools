#!/usr/bin/env python3
"""The ghost and the arrow: where a part came FROM (#946, #1020).

A placement tween glides parts from their source pose to their parsed one, and
that reads as *the board assembling itself* rather than as *these eleven parts
moved, from there to here*. Watched frame by frame, a viewer sees where a part
ARRIVED and never where it came from -- which is the one question a placement
film exists to answer.

`py_tools/render_placement` has drawn a ghost and an arrow for a STILL since
#896. It could not be shared, because the still draws from a `--before` board
and the film has no such thing: the movie re-points `renderer.pcb` at
interpolated poses, so the "before" exists only as an offset inside
`movie_camera.Stage._tween`. That offset is exactly what this takes.

**THROUGH THE `overlays=` SEAM, so it costs NO FRAME GEOMETRY.**
`route_render.frame(overlays=[fn(draw, renderer)])` draws at supersampled
resolution above the copper and below the label. The frame size, the frame
count and every pinned geometry assertion are untouched; this is decoration in
the literal sense, which is also why it may never raise.

**THE ARROW GROWS AND THE GHOST FADES IN**, which is the opposite of the first
version of both and the reason is the same for each: the annotation must be
strongest when it is the ONLY record of where the part came from.

The arrow runs from the ghost to the part's CURRENT pose, so it starts at zero
length and stretches as the part travels -- at the end of the glide it spans
the whole journey. And the ghost therefore has to fade IN, not out: at t=0 it
coincides with the part and says nothing (it is suppressed outright), while at
t=1 the part has gone and the ghost is the only thing marking the origin. The
first version faded it OUT, which made it faintest exactly when the arrow was
longest and the information most needed.

**A PART THAT BARELY MOVED GETS NOTHING.** Below `MIN_TRAVEL_MM` the ghost and
the part overlap, so the ghost is a smear on the part and the arrow is a dot.
`render_placement`'s own arrow rule is the precedent.
"""
from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: Below this much travel the ghost overlaps the part and says nothing.
#: `render_placement` refuses an arrow on the same grounds.
MIN_TRAVEL_MM = 1.5

#: The ghost's strength against the board at the start of the glide and at the
#: end. It fades IN: at the start it sits on top of the part and says nothing,
#: and at the end it is the only thing marking where the part came from.
GHOST_FADE = (0.25, 0.90)


def _lerp(a, b, t):
    return a + (b - a) * t


def ghost_overlay(items, theme, t=0.0):
    """`fn(draw, renderer)` drawing a ghost and an arrow per moved part.

    `items` is `[(ref, from_xy, now_xy, half_w, half_h, pad_offset), ...]`
    in BOARD mm, where the poses are footprint ORIGINS and `pad_offset` is
    where the pad bbox sits relative to one --
    the caller owns the poses, because only it knows what "from" means. `t` is
    how far through the glide we are, 0..1, and only the fade uses it.

    Never raises: a ghost is decoration, and losing a movie to one trades a
    cosmetic problem for a real one.
    """
    def _draw(d, r):
        try:
            import render_theme
            th = theme or render_theme.DARK
            ghost = th.rgb('place_ghost')
            arrow = th.rgb('place_arrow')
            ss = max(1, int(getattr(r, 'ss', 1)))
            fade = _lerp(GHOST_FADE[0], GHOST_FADE[1],
                         max(0.0, min(1.0, float(t))))
            body = th.rgb('board_body')
            # The ghost is composited by hand against the board, because the
            # overlay draws onto an RGB canvas: an alpha here would be a lie
            # about what lands.
            gcol = tuple(int(round(body[i] + (ghost[i] - body[i]) * fade))
                         for i in range(3))
            for it in items:
                ref, (fx, fy), (nx, ny), hw, hh = it[:5]
                ox, oy = it[5] if len(it) > 5 else (0.0, 0.0)
                travel = math.hypot(nx - fx, ny - fy)
                if travel < MIN_TRAVEL_MM:
                    continue
                p0 = r.tf.pt(fx + ox - hw, fy + oy - hh)
                p1 = r.tf.pt(fx + ox + hw, fy + oy + hh)
                box = [min(p0[0], p1[0]), min(p0[1], p1[1]),
                       max(p0[0], p1[0]), max(p0[1], p1[1])]
                if box[2] - box[0] < 1 or box[3] - box[1] < 1:
                    continue
                d.rectangle(box, outline=gcol, width=max(1, ss))
                a = r.tf.pt(fx, fy)
                b = r.tf.pt(nx, ny)
                if math.hypot(b[0] - a[0], b[1] - a[1]) < 3 * ss:
                    continue
                # HALOED, because the arrow crosses copper it is close to.
                # Measured: `place_arrow` vs `pad` is 65.2 apart on dark and
                # **44.7 on light** (40.1 deuteranope) -- barely above the 34
                # this issue treats as indistinguishable, on a 1-2 px line
                # drawn straight across the pads. A darker stroke underneath
                # separates it from whatever it crosses without touching a
                # single palette value, which is this issue's own lesson: give
                # it a second channel rather than a new colour.
                ang = math.atan2(b[1] - a[1], b[0] - a[0])
                hl = max(4, 5 * ss)
                head = [(b[0] + hl * math.cos(ang + k),
                         b[1] + hl * math.sin(ang + k)) for k in (2.6, -2.6)]
                for col, wid in ((body, max(3, 3 * ss)),
                                 (arrow, max(1, ss))):
                    d.line([a[0], a[1], b[0], b[1]], fill=col, width=wid)
                    for hx, hy in head:
                        d.line([b[0], b[1], hx, hy], fill=col, width=wid)
        except Exception:                                      # noqa: BLE001
            pass       # a ghost is never worth failing a render over
    return _draw


def items_from_deltas(deltas, home):
    """`movie_camera.Stage._tween`'s own state -> `ghost_overlay` items.

    `deltas` is `[(ref, footprint, dx, dy), ...]` where `(dx, dy)` is
    `from - to`, and `home[ref]` carries the pads at their PARSED (destination)
    poses. So the source pose is the home centre plus the delta, and the extent
    is the home pad bbox -- which is the part's own footprint rather than a
    guess at its size.
    """
    out = []
    for ref, fp, dx, dy in deltas:
        rec = home.get(ref)
        hx, hy = (rec[0], rec[1]) if rec else (fp.x, fp.y)
        pads = rec[2] if rec and len(rec) > 2 else []
        xs = [gx for _p, gx, _gy in pads]
        ys = [gy for _p, _gx, gy in pads]
        if xs and ys:
            hw = max((max(xs) - min(xs)) / 2.0, 0.2)
            hh = max((max(ys) - min(ys)) / 2.0, 0.2)
            # The pad bbox is not centred on the footprint ORIGIN, so carry
            # its offset rather than mixing the two frames of reference: the
            # arrow runs origin-to-origin and the ghost sits where the copper
            # actually would.
            ox = (max(xs) + min(xs)) / 2.0 - hx
            oy = (max(ys) + min(ys)) / 2.0 - hy
        else:
            hw = hh = 0.5
            ox = oy = 0.0
        out.append((ref, (hx + dx, hy + dy), (fp.x, fp.y), hw, hh,
                    (ox, oy)))
    return out
