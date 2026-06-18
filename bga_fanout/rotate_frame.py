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
import copy
import math
from typing import Callable, Tuple

from kicad_parser import PCBData


def is_orthogonal(rotation_deg: float, tol: float = 1.0) -> bool:
    """True if the footprint sits at (near) a multiple of 90° - the grid stays
    axis-aligned in global space and no frame transform is needed."""
    r = rotation_deg % 90.0
    return r <= tol or r >= 90.0 - tol


def _fold_angle(a: float) -> float:
    a %= 180.0
    return a if a <= 90.0 else a - 180.0


def to_axis_aligned_frame(pcb_data: PCBData, ref: str
                          ) -> Tuple[PCBData, Callable[[float, float], Tuple[float, float]]]:
    """Return (rotated_pcb, back_transform).

    `rotated_pcb` is a deep copy of `pcb_data` rotated about footprint `ref`'s
    centre so that `ref` becomes axis-aligned (its `rotation` is set to 0 and its
    pads land on their local grid). `back_transform(x, y)` maps a point from the
    rotated frame back to the real board frame. Pad sizes are unchanged; pad
    `rect_rotation` is carried along so foreign rectangular pads keep their shape.
    """
    fp = pcb_data.footprints[ref]
    cx, cy = fp.x, fp.y
    theta = fp.rotation

    # Forward (real -> rotated): undo the footprint rotation. local_to_global
    # applies R(-theta), so the inverse is R(+theta) about the centre.
    fr = math.radians(theta)
    fcos, fsin = math.cos(fr), math.sin(fr)

    def fwd(x, y):
        dx, dy = x - cx, y - cy
        return (cx + fcos * dx - fsin * dy, cy + fsin * dx + fcos * dy)

    # Back (rotated -> real): R(-theta).
    br = math.radians(-theta)
    bcos, bsin = math.cos(br), math.sin(br)

    def back(x, y):
        dx, dy = x - cx, y - cy
        return (cx + bcos * dx - bsin * dy, cy + bsin * dx + bcos * dy)

    rp = copy.deepcopy(pcb_data)

    seen_pads = set()

    def _xform_pad(pad):
        if id(pad) in seen_pads:
            return
        seen_pads.add(id(pad))
        pad.global_x, pad.global_y = fwd(pad.global_x, pad.global_y)
        # The whole world rotates by +theta, so each pad's tilt does too.
        pad.rect_rotation = _fold_angle(pad.rect_rotation + theta)

    # The TARGET footprint's pads land on their exact axis-aligned grid
    # (cx + local_x, cy + local_y). Snap them there rather than computing through
    # the rotation: at non-orthogonal angles the trig leaves same-column balls
    # with sub-micron x differences, and the grid analysis groups by exact value.
    # back() maps these snapped points to the true ball positions.
    for pad in rp.footprints[ref].pads:
        pad.global_x = cx + pad.local_x
        pad.global_y = cy + pad.local_y
        pad.rect_rotation = _fold_angle(pad.rect_rotation + theta)
        seen_pads.add(id(pad))

    for f in rp.footprints.values():
        for pad in f.pads:
            _xform_pad(pad)
    for plist in rp.pads_by_net.values():
        for pad in plist:
            _xform_pad(pad)

    for seg in rp.segments:
        seg.start_x, seg.start_y = fwd(seg.start_x, seg.start_y)
        seg.end_x, seg.end_y = fwd(seg.end_x, seg.end_y)

    for via in rp.vias:
        via.x, via.y = fwd(via.x, via.y)

    # The target footprint is now axis-aligned.
    rp.footprints[ref].rotation = 0.0

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
