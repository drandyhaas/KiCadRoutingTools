"""The board turned over, so a BACK-side BGA fans out as the mirror of a
front-side one.

`bga_fanout` was written for a part on F.Cu -- the surface escape is the
pad's layer, the dog-bone dives to the other, the edge escapes are named
up/down/left/right in board axes -- and measured on a board and its
mirror (the same chip on B), 18 of 51 escapes differed: the gaps along a
face assigned in the other order, one net on the other layer with a via
the front did not need. A chip on the back should fan out exactly as
the same chip on the front, mirrored.

Rather than find every place the engine leans on F, this does for the
face what `rotate_frame.py` does for the angle: turn the whole board
over in memory (every part to the other face, y mirrored about the
board's centre line, every layer swapped, pad locals re-derived under
the parser's own convention), run the engine on the part now on F, and
mirror the copper back. The mirror is a half turn in 3D, an isometry of
every clearance rule, so what is legal on the turned-over board is
legal on the real one. A part on F skips this entirely and behaves
exactly as before; the rotation wrapper still applies after it.
"""
from __future__ import annotations

import copy
from typing import Callable, Dict, Tuple

from kicad_parser import PCBData, Footprint, _global_to_local

Point = Tuple[float, float]


def is_back_side(footprint: Footprint) -> bool:
    return str(footprint.layer or '').startswith('B')


def other_layer(name):
    """F.* <-> B.*; anything else (inner layers, user layers) unchanged."""
    if not isinstance(name, str) or len(name) < 2 or name[1] != '.':
        return name
    if name[0] == 'F':
        return 'B' + name[1:]
    if name[0] == 'B':
        return 'F' + name[1:]
    return name


def _fold(a: float) -> float:
    a %= 180.0
    return a if a <= 90.0 else a - 180.0


AXIS_LATTICE = 0.05   # 2 * axis a multiple of 0.1: every routing grid step
                      # that divides 0.1 mm (0.1, 0.05, 0.025) is its own mirror


def mirror_axis(pcb_data: PCBData) -> float:
    """The board's horizontal centre line, on the lattice: the bounds'
    middle (the pads' middle when there are none), rounded so that twice
    the axis is a multiple of 0.1 mm. The routers' grids are anchored at
    the origin, so a mirror about an arbitrary line maps the grid onto a
    shifted grid and the turned board routes differently in the last
    cell (measured: an array on the back fanned out with 0.2 mm jogs its
    front twin did not have); about a lattice line the grid is its own
    mirror and the copper is exact."""
    b = pcb_data.board_info.board_bounds if pcb_data.board_info else None
    if b:
        c = (b[1] + b[3]) / 2.0
    else:
        ys = [p.global_y for f in pcb_data.footprints.values() for p in f.pads]
        c = (min(ys) + max(ys)) / 2.0 if ys else 0.0
    return round(c / AXIS_LATTICE) * AXIS_LATTICE


def to_front_frame(pcb_data: PCBData, ref: str, axis: float = None
                   ) -> Tuple[PCBData, Callable[[float, float], Point]]:
    """Return (turned_pcb, back). `turned_pcb` is a deep copy of the board
    turned over about its centre line: `ref` (and every other part) on
    the other face. `back(x, y)` maps a point of the turned board to the
    real one (the mirror is its own inverse).

    `axis` -- the mirror line's y; default `mirror_axis` (the board's
    centre line on the lattice). The board bounds are mirrored about it
    like everything else."""
    CY = mirror_axis(pcb_data) if axis is None else float(axis)

    def m(x, y):
        return (x, 2.0 * CY - y)

    rp = copy.deepcopy(pcb_data)
    # the turned copy says so: a memo keyed on the board FILE (path,
    # mtime, size) would otherwise hand the turned board the real one's
    # model -- measured: the braid's obstacle memo gave the turned
    # mirror an F.Cu model of 22 discs where the front had 473
    rp.frame_axis = CY
    seen = set()

    def xform_pad(pad):
        if id(pad) in seen:
            return
        seen.add(id(pad))
        pad.global_x, pad.global_y = m(pad.global_x, pad.global_y)
        if pad.hole_x is not None and pad.hole_y is not None:
            pad.hole_x, pad.hole_y = m(pad.hole_x, pad.hole_y)
        pad.layers = [other_layer(L) for L in pad.layers]
        pad.rotation = (-pad.rotation) % 360.0
        pad.rect_rotation = _fold(-pad.rect_rotation)

    for f in rp.footprints.values():
        f.y = 2.0 * CY - f.y
        f.rotation = (-f.rotation) % 360.0
        f.layer = other_layer(f.layer)
        for pad in f.pads:
            xform_pad(pad)
        # pad locals under the parser's own convention (the exact inverse
        # of local_to_global; the B-side pre-mirroring folds into local)
        for pad in f.pads:
            pad.local_x, pad.local_y = _global_to_local(f.x, f.y, f.rotation,
                                                        pad.global_x, pad.global_y)
    for plist in rp.pads_by_net.values():
        for pad in plist:
            xform_pad(pad)
    for net in rp.nets.values():
        for pad in getattr(net, 'pads', ()) or ():
            xform_pad(pad)
    for seg in rp.segments:
        seg.start_x, seg.start_y = m(seg.start_x, seg.start_y)
        seg.end_x, seg.end_y = m(seg.end_x, seg.end_y)
        seg.layer = other_layer(seg.layer)
    for via in rp.vias:
        via.x, via.y = m(via.x, via.y)
        via.layers = [other_layer(L) for L in via.layers]
    for z in rp.zones or []:
        z.layer = other_layer(z.layer)
        z.polygon = [m(x, y) for (x, y) in z.polygon]
    for gp in list(rp.guide_paths or []) + list(rp.keepout_zones or []):
        if hasattr(gp, 'layer'):
            gp.layer = other_layer(gp.layer)
        if hasattr(gp, 'points'):
            gp.points = [m(x, y) for (x, y) in gp.points]
    bi = rp.board_info
    if bi is not None:
        if bi.board_bounds:
            x0, y0, x1, y1 = bi.board_bounds
            bi.board_bounds = (x0, 2.0 * CY - y1, x1, 2.0 * CY - y0)
        for attr in ('board_outline',):
            pts = getattr(bi, attr, None)
            if pts:
                setattr(bi, attr, [m(x, y) for (x, y) in pts])
        for attr in ('board_cutouts', 'board_outlines', 'board_edge_contours'):
            polys = getattr(bi, attr, None)
            if polys:
                setattr(bi, attr, [[m(x, y) for (x, y) in poly] for poly in polys])
        for k in list(getattr(bi, 'keepouts', None) or []):
            if isinstance(k, dict):
                for key in ('polygon', 'points'):
                    if isinstance(k.get(key), list):
                        k[key] = [m(x, y) for (x, y) in k[key]]
                for key in ('layer', 'layers'):
                    v = k.get(key)
                    if isinstance(v, str):
                        k[key] = other_layer(v)
                    elif isinstance(v, list):
                        k[key] = [other_layer(L) for L in v]
    return rp, m


_FLIP_FACE = {'up': 'down', 'down': 'up', 'left': 'left', 'right': 'right'}


def flip_hints(hints: Dict, footprint: Footprint, turned: PCBData,
               m: Callable[[float, float], Point]) -> Dict:
    """escape_dir_hints are keyed by BOARD pad position and name BOARD
    faces and layers: re-key each to the turned pad, swap up/down, mirror
    a full move's exit and via site, swap its layer."""
    if not hints:
        return hints
    turned_pads = {p.pad_number: p for p in turned.footprints[footprint.reference].pads}
    out = {}
    for p in footprint.pads:
        d = hints.get((round(p.global_x, 3), round(p.global_y, 3)))
        q = turned_pads.get(p.pad_number)
        if d is None or q is None:
            continue
        if isinstance(d, dict):
            mv = dict(d)
            if mv.get('face') in _FLIP_FACE:
                mv['face'] = _FLIP_FACE[mv['face']]
            if mv.get('exit') is not None:
                mv['exit'] = m(*mv['exit'])
            if mv.get('site') is not None:
                mv['site'] = m(*mv['site'])
            if mv.get('path'):
                mv['path'] = [m(*q) for q in mv['path']]
            if isinstance(mv.get('layer'), str):
                mv['layer'] = other_layer(mv['layer'])
            d = mv
        elif d in _FLIP_FACE:
            d = _FLIP_FACE[d]
        out[(round(q.global_x, 3), round(q.global_y, 3))] = d
    return out


def flip_results(tracks, vias_to_add, vias_to_remove,
                 m: Callable[[float, float], Point]):
    """Map fanout tracks and vias from the turned board back to the real
    one, in place: y mirrored, layers swapped."""
    for t in tracks:
        t['start'] = m(*t['start'])
        t['end'] = m(*t['end'])
        if isinstance(t.get('layer'), str):
            t['layer'] = other_layer(t['layer'])
    for v in list(vias_to_add) + list(vias_to_remove):
        v['x'], v['y'] = m(v['x'], v['y'])
        if isinstance(v.get('layers'), list):
            v['layers'] = [other_layer(L) for L in v['layers']]
    return tracks, vias_to_add, vias_to_remove
