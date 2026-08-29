"""Flow frame: run the corridor pipeline in a frame where the source is
due WEST of the destination, whatever their real relative placement.

The braid hard-wires "source west, destination east": x is time, y is
lane, and every slot / morph / window / street computation is written in
those terms. Rewriting all of it is a large, risky change for no gain,
and the repo already has the answer for exactly this shape of problem --
`bga_fanout/rotate_frame.py` rotates a non-orthogonal BGA into its own
frame, runs the unchanged pipeline, and rotates the tracks back.

Same trick here. The flow angle is snapped to a multiple of 90 degrees,
which matters for two reasons:

  * octilinear geometry in the rotated frame is octilinear in the board
    frame EXACTLY (true for multiples of 45, but see below);
  * the entry logic reasons about ROW streets and COLUMN gaps of the
    destination array. At 45 degrees those stop being rows and columns,
    so the grid analysis would break. At 90 they are simply relabelled.

Four cardinal directions is also what orthogonal placement -- the normal
case -- actually produces. A diagonal placement snaps to its nearest
cardinal: the corridor is then less well aligned with the flow, but
every computation stays valid.

A source already due west gives angle 0 and this module is a NO-OP, so
adopting it cannot change an existing result.
"""
from __future__ import annotations

import copy
import math
from typing import Callable, Iterable, Tuple

from kicad_parser import PCBData


def flow_angle(src_pts: Iterable[Tuple[float, float]],
               dst_pts: Iterable[Tuple[float, float]]) -> float:
    """Rotation (degrees, a multiple of 90) that carries the
    source->destination direction onto +x, i.e. what the pipeline must
    apply to make the source lie due west of the destination."""
    sx = list(src_pts)
    dx = list(dst_pts)
    if not sx or not dx:
        return 0.0
    cs = (sum(p[0] for p in sx) / len(sx), sum(p[1] for p in sx) / len(sx))
    cd = (sum(p[0] for p in dx) / len(dx), sum(p[1] for p in dx) / len(dx))
    ang = math.degrees(math.atan2(cd[1] - cs[1], cd[0] - cs[0]))
    # snap to the nearest cardinal, then report the rotation that undoes it
    snapped = round(ang / 90.0) * 90.0
    return (-snapped) % 360.0


def rotate_pcb(pcb_data: PCBData, theta_deg: float,
               cx: float, cy: float
               ) -> Tuple[PCBData, Callable[[float, float],
                                            Tuple[float, float]]]:
    """Return (rotated copy, back_transform) for a rigid rotation of
    `theta_deg` about (cx, cy). `back(x, y)` maps a point from the
    rotated frame to the real board frame. The transform is rigid, so
    every clearance and collision computed in the rotated frame stays
    valid when mapped back."""
    fr = math.radians(theta_deg)
    fcos, fsin = math.cos(fr), math.sin(fr)

    def fwd(x, y):
        dx, dy = x - cx, y - cy
        return (cx + fcos * dx - fsin * dy, cy + fsin * dx + fcos * dy)

    br = math.radians(-theta_deg)
    bcos, bsin = math.cos(br), math.sin(br)

    def back(x, y):
        dx, dy = x - cx, y - cy
        return (cx + bcos * dx - bsin * dy, cy + bsin * dx + bcos * dy)

    if abs(theta_deg % 360.0) < 1e-9:
        return pcb_data, back            # exact no-op, no deep copy

    rp = copy.deepcopy(pcb_data)
    seen = set()

    def _pad(pad):
        if id(pad) in seen:
            return
        seen.add(id(pad))
        pad.global_x, pad.global_y = fwd(pad.global_x, pad.global_y)
        if pad.hole_x is not None and pad.hole_y is not None:
            pad.hole_x, pad.hole_y = fwd(pad.hole_x, pad.hole_y)
        # a 90-degree world rotation swaps a pad's own extents
        if round(theta_deg / 90.0) % 2 == 1:
            pad.size_x, pad.size_y = pad.size_y, pad.size_x

    for f in rp.footprints.values():
        f.x, f.y = fwd(f.x, f.y)
        f.rotation = (f.rotation - theta_deg) % 360.0
        for pad in f.pads:
            _pad(pad)
    for plist in getattr(rp, 'pads_by_net', {}).values():
        for pad in plist:
            _pad(pad)
    for seg in rp.segments:
        seg.start_x, seg.start_y = fwd(seg.start_x, seg.start_y)
        seg.end_x, seg.end_y = fwd(seg.end_x, seg.end_y)
    for via in rp.vias:
        via.x, via.y = fwd(via.x, via.y)
    if rp.board_info and rp.board_info.board_bounds:
        x0, y0, x1, y1 = rp.board_info.board_bounds
        corners = [fwd(x0, y0), fwd(x1, y0), fwd(x0, y1), fwd(x1, y1)]
        rp.board_info.board_bounds = (min(c[0] for c in corners),
                                      min(c[1] for c in corners),
                                      max(c[0] for c in corners),
                                      max(c[1] for c in corners))
    return rp, back
