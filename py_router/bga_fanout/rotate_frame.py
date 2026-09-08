"""Rigid-frame transform so bga_fanout can handle non-orthogonally-placed BGAs.

`bga_fanout` analyses the ball grid and routes escapes entirely in the global
board axes (rows = global Y, cols = global X, channels horizontal/vertical, edge
escapes left/right/up/down). That only works when the part sits at a multiple of
90°; at any other angle the balls form a diamond and the grid/escape logic breaks
(issue #137).

Rather than rewrite every axis assumption, we rotate the whole problem into the
footprint's own frame - where its pads ARE axis-aligned - run the existing
pipeline unchanged, then rotate the produced tracks/vias back. The transform is a
rigid rotation about the footprint centre, so all clearances/collisions computed
in the rotated frame remain valid when mapped back.

Only used for non-orthogonal placements; orthogonal BGAs skip this entirely and
behave exactly as before.
"""
from __future__ import annotations

import copy
import math
import os
from typing import Callable, Tuple

from kicad_parser import PCBData


def is_orthogonal(rotation_deg: float, tol: float = 1.0) -> bool:
    """True if the footprint sits at (near) a multiple of 90°."""
    r = rotation_deg % 90.0
    return r <= tol or r >= 90.0 - tol


def needs_frame(rotation_deg: float, tol: float = 1.0) -> bool:
    """Whether the engine routes this part in its own frame. Non-orthogonal
    angles always (issue #137: the grid must be axis-aligned). Quarter
    turns only on request (`KICAD_FANOUT_FRAME_QUARTER=1`): the engine's
    escape order and faces are written in board axes, so the same array
    at 0 / 90 / 180 / 270 degrees fanned out with 502 / 540 / 425 / 430
    tracks and braided to 38 / 42 / 54 / 38 vias (#622 pose gate,
    2026-09-08); in its own frame the half turn is exact and the quarter
    turns agree with each other, but the tuned bench's destination array
    sits at 90 degrees and took a different draw there (K15 14 = 14,
    K28 38 -> 36, K41 82 -> 106 vias), so the frame is not the default
    for quarter turns. Measured, not assumed; the switch reproduces it."""
    r = rotation_deg % 360.0
    if r <= tol or r >= 360.0 - tol:
        return False
    if is_orthogonal(rotation_deg):
        return os.environ.get('KICAD_FANOUT_FRAME_QUARTER', '') == '1'
    return True


LATTICE = 0.05   # a quarter turn about a lattice point keeps every routing
                 # grid step that divides 0.05 mm its own image


def _fold_angle(a: float) -> float:
    a %= 180.0
    return a if a <= 90.0 else a - 180.0


def _frame_of(fp):
    """(cx, cy, theta, fwd, back) for the footprint: the rotation that
    makes it axis-aligned. A quarter-turn part turns EXACTLY (no trig)
    about the lattice point nearest its centre, so the origin-anchored
    routing grids are their own image; any other angle turns about the
    part's own centre with trig, as before."""
    theta = fp.rotation
    if is_orthogonal(theta):
        k = int(round(theta / 90.0)) % 4
        cx = round(fp.x / LATTICE) * LATTICE
        cy = round(fp.y / LATTICE) * LATTICE

        def _q(dx, dy, k):
            # R(+90 k) in the parser's convention: (dx, dy) -> (-dy, dx)
            for _ in range(k):
                dx, dy = -dy, dx
            return dx, dy

        def fwd(x, y):
            dx, dy = _q(x - cx, y - cy, k)
            return (cx + dx, cy + dy)

        def back(x, y):
            dx, dy = _q(x - cx, y - cy, (4 - k) % 4)
            return (cx + dx, cy + dy)
        return cx, cy, 90.0 * k, fwd, back
    cx, cy = fp.x, fp.y
    fr = math.radians(theta)
    fcos, fsin = math.cos(fr), math.sin(fr)
    br = math.radians(-theta)
    bcos, bsin = math.cos(br), math.sin(br)

    def fwd(x, y):
        dx, dy = x - cx, y - cy
        return (cx + fcos * dx - fsin * dy, cy + fsin * dx + fcos * dy)

    def back(x, y):
        dx, dy = x - cx, y - cy
        return (cx + bcos * dx - bsin * dy, cy + bsin * dx + bcos * dy)
    return cx, cy, theta, fwd, back


def forward_transform(pcb_data: PCBData, ref: str
                      ) -> Callable[[float, float], Tuple[float, float]]:
    """The real -> rotated point map to_axis_aligned_frame applies to `ref`'s
    pads (the same rotation about the same centre), for callers that must
    carry their own board points -- a planned escape's exit or via site --
    into the routing frame."""
    return _frame_of(pcb_data.footprints[ref])[3]


def to_axis_aligned_frame(pcb_data: PCBData, ref: str
                          ) -> Tuple[PCBData, Callable[[float, float], Tuple[float, float]]]:
    """Return (rotated_pcb, back_transform).

    `rotated_pcb` is a deep copy of `pcb_data` rotated so that `ref` becomes
    axis-aligned (its `rotation` is set to 0 and its pads land on their local
    grid). `back_transform(x, y)` maps a point from the rotated frame back to
    the real board frame. Pad sizes are unchanged; pad `rect_rotation` is
    carried along so foreign rectangular pads keep their shape. Every other
    part, the drill holes, the zones, keep-outs, guide paths and the board
    bounds turn with the pads, so the frame is the whole board.
    """
    fp = pcb_data.footprints[ref]
    cx, cy, theta, fwd, back = _frame_of(fp)
    ortho = is_orthogonal(theta)

    rp = copy.deepcopy(pcb_data)
    rp.frame_rotation = theta     # a memo keyed on the board FILE must see another board

    seen_pads = set()

    odd = ortho and int(round(theta / 90.0)) % 2 == 1

    def _xform_pad(pad):
        if id(pad) in seen_pads:
            return
        seen_pads.add(id(pad))
        pad.global_x, pad.global_y = fwd(pad.global_x, pad.global_y)
        if pad.hole_x is not None and pad.hole_y is not None:
            pad.hole_x, pad.hole_y = fwd(pad.hole_x, pad.hole_y)
        if ortho:
            # the parser's own convention for a pad at a quarter turn:
            # board-space sizes swapped, the residual tilt unchanged --
            # the representation a board drawn this way round would
            # parse to, so every consumer's fast path sees what it
            # would see on that board
            if odd:
                pad.size_x, pad.size_y = pad.size_y, pad.size_x
            # KiCad's angle runs the other way from the geometric turn
            # (local_to_global applies R(-rotation)): the target goes from
            # theta to 0, so everything's angle changes by -theta
            pad.rotation = (pad.rotation - theta) % 360.0
        else:
            # The whole world rotates by +theta, so each pad's tilt does too.
            pad.rect_rotation = _fold_angle(pad.rect_rotation + theta)

    tgt = rp.footprints[ref]
    if not ortho:
        # The TARGET footprint's pads land on their exact axis-aligned grid
        # (cx + local_x, cy + local_y). Snap them there rather than computing
        # through the rotation: at non-orthogonal angles the trig leaves
        # same-column balls with sub-micron x differences, and the grid
        # analysis groups by exact value. back() maps these snapped points to
        # the true ball positions.
        for pad in tgt.pads:
            pad.global_x = cx + pad.local_x
            pad.global_y = cy + pad.local_y
            if pad.hole_x is not None and pad.hole_y is not None:
                pad.hole_x, pad.hole_y = fwd(pad.hole_x, pad.hole_y)
            pad.rect_rotation = _fold_angle(pad.rect_rotation + theta)
            seen_pads.add(id(pad))
    # (a quarter-turn part's pads come through the exact rotation with
    # everything else: same-column balls stay equal to the bit)

    for f in rp.footprints.values():
        for pad in f.pads:
            _xform_pad(pad)
        if f is not tgt:
            f.x, f.y = fwd(f.x, f.y)
            f.rotation = (f.rotation - theta) % 360.0
    for plist in rp.pads_by_net.values():
        for pad in plist:
            _xform_pad(pad)
    for net in rp.nets.values():
        for pad in getattr(net, 'pads', ()) or ():
            _xform_pad(pad)

    for seg in rp.segments:
        seg.start_x, seg.start_y = fwd(seg.start_x, seg.start_y)
        seg.end_x, seg.end_y = fwd(seg.end_x, seg.end_y)

    for via in rp.vias:
        via.x, via.y = fwd(via.x, via.y)

    for z in rp.zones or []:
        z.polygon = [fwd(x, y) for (x, y) in z.polygon]
    for gp in list(rp.guide_paths or []) + list(rp.keepout_zones or []):
        if hasattr(gp, 'points'):
            gp.points = [fwd(x, y) for (x, y) in gp.points]
    bi = rp.board_info
    if bi is not None:
        if bi.board_bounds:
            x0, y0, x1, y1 = bi.board_bounds
            cs = [fwd(x0, y0), fwd(x1, y0), fwd(x0, y1), fwd(x1, y1)]
            bi.board_bounds = (min(c[0] for c in cs), min(c[1] for c in cs),
                               max(c[0] for c in cs), max(c[1] for c in cs))
        for attr in ('board_outline',):
            pts = getattr(bi, attr, None)
            if pts:
                setattr(bi, attr, [fwd(x, y) for (x, y) in pts])
        for attr in ('board_cutouts', 'board_outlines', 'board_edge_contours'):
            polys = getattr(bi, attr, None)
            if polys:
                setattr(bi, attr, [[fwd(x, y) for (x, y) in poly] for poly in polys])
        for k in list(getattr(bi, 'keepouts', None) or []):
            if isinstance(k, dict):
                for key in ('polygon', 'points'):
                    if isinstance(k.get(key), list):
                        k[key] = [fwd(x, y) for (x, y) in k[key]]

    # The target footprint is now axis-aligned, at its turned centre
    # (its own centre unless the quarter turn went about the lattice
    # point beside it); its pads' locals are unchanged by construction.
    if ortho:
        tgt.x, tgt.y = fwd(fp.x, fp.y)
    tgt.rotation = 0.0

    return rp, back


def back_transform_results(tracks, vias_to_add, vias_to_remove,
                           back: Callable[[float, float], Tuple[float, float]]):
    """Map fanout tracks (start/end) and via positions (x/y) from the rotated
    frame back to the real board frame, in place."""
    for t in tracks:
        t['start'] = back(*t['start'])
        t['end'] = back(*t['end'])
    for v in vias_to_add:
        v['x'], v['y'] = back(v['x'], v['y'])
    for v in vias_to_remove:
        v['x'], v['y'] = back(v['x'], v['y'])
