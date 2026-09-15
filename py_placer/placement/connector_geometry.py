"""Drawn connector envelope relative to a declared compass board edge (#961).

The mating face is the support line of the drawn envelope in the declared
outward direction. Signed position is positive outside; overhang=max(0,signed)
and setback=max(0,-signed). Stroke widths, text, pads and courtyards are not
body geometry. This is a 2-D drawing contract, not a 3-D mating simulation.

Supported bodies have a closed convex polygonal envelope on the footprint's
own Fab layer, else its own SilkS layer. Internal line markings are permitted;
every convex-hull side must actually be drawn. Open ticks, concave envelopes,
curves and opposite-face-only drawings are explicitly unmeasured. Vertices
are transformed before taking extrema, so arbitrary rotations are exact.

The boundary contract currently supports an actual rectangular Edge.Cuts
outline without other cuts. Concavity, cutouts and open/curved boundaries are
unmeasured, never approximated by the board box or a sum of violations.
"""
from __future__ import annotations

import math
import re

from kicad_parser import find_matching_paren
from .parser import _footprint_blocks

EPS = 1e-6
EDGES = ('west', 'east', 'north', 'south')


def _point(text, name):
    m = re.search(r'\(' + name + r'\s+([-+\d.eE]+)\s+([-+\d.eE]+)\)', text)
    if not m:
        raise ValueError('missing ' + name)
    p = tuple(float(v) for v in m.groups())
    if not all(math.isfinite(v) for v in p):
        raise ValueError('nonfinite body coordinate')
    return p


def _cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def _envelope(segments):
    points = sorted(set(p for s in segments for p in s))
    if len(points) < 3:
        raise ValueError('body has no closed polygonal envelope')
    def half(seq):
        out = []
        for p in seq:
            while len(out) > 1 and _cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out
    hull = half(points)[:-1] + half(points[::-1])[:-1]
    if len(hull) < 3:
        raise ValueError('degenerate body drawing')
    for a, b in zip(hull, hull[1:] + hull[:1]):
        length = math.dist(a, b)
        runs = []
        for c, d in segments:
            if abs(_cross(a, b, c))/length > EPS or abs(_cross(a, b, d))/length > EPS:
                continue
            runs.append(sorted(((p[0]-a[0])*(b[0]-a[0])/length
                                + (p[1]-a[1])*(b[1]-a[1])/length for p in (c, d))))
        end = 0.0
        for lo, hi in sorted(runs):
            if lo > end + EPS:
                break
            end = max(end, hi)
        if end < length - EPS:
            raise ValueError('open or concave body envelope (convex hull side is not drawn)')
    return hull


class ConnectorGeometry:
    def __init__(self, pcb_data, path):
        self.pcb = pcb_data
        self.bodies = {}
        self.boundary_reason = ''
        self.bounds = getattr(pcb_data.board_info, 'board_bounds', None)
        try:
            with open(path, encoding='utf-8') as stream:
                text = stream.read()
            from kicad_parser import _collect_edge_cuts_segments
            from .legality import _segments_cover_rectangle
            if not self.bounds or not _segments_cover_rectangle(
                    _collect_edge_cuts_segments(text), self.bounds):
                self.boundary_reason = ('unsupported boundary: requires rectangular Edge.Cuts '
                                        'without concavity, cutouts, curves or extra open cuts')
            for ref, block in _footprint_blocks(text):
                fp = pcb_data.footprints.get(ref)
                if fp is None:
                    continue
                # Local coordinates in a saved B footprint are already mirrored.
                side = 'B' if fp.layer.startswith('B.') else 'F'
                layers = {}
                for m in re.finditer(r'\(fp_(\w+)\b', block):
                    kind = m.group(1)
                    if kind in ('text', 'text_box'):
                        continue
                    item = block[m.start():find_matching_paren(block, m.start())]
                    lm = re.search(r'\(layer\s+"([FB]\.(?:Fab|SilkS))"\)', item)
                    if lm:
                        layers.setdefault(lm.group(1), []).append((kind, item))
                layer = next((side + suffix for suffix in ('.Fab', '.SilkS')
                              if side + suffix in layers), None)
                if layer is None:
                    self.bodies[ref] = (None, None, 'no body drawing on the footprint face')
                    continue
                try:
                    segments = []
                    for kind, item in layers[layer]:
                        if kind == 'line':
                            segments.append((_point(item, 'start'), _point(item, 'end')))
                        elif kind == 'rect':
                            a, b = _point(item, 'start'), _point(item, 'end')
                            pts = [a, (b[0], a[1]), b, (a[0], b[1])]
                            segments.extend(zip(pts, pts[1:] + pts[:1]))
                        elif kind == 'poly':
                            pts = [_point(x.group(), 'xy') for x in re.finditer(
                                r'\(xy\s+[-+\d.eE]+\s+[-+\d.eE]+\)', item)]
                            segments.extend(zip(pts, pts[1:] + pts[:1]))
                        else:
                            raise ValueError('unsupported body primitive: fp_' + kind)
                    self.bodies[ref] = (_envelope(segments), layer, '')
                except ValueError as exc:
                    self.bodies[ref] = (None, layer, str(exc))
        except (OSError, TypeError, ValueError) as exc:
            self.boundary_reason = 'body/boundary source unreadable: ' + str(exc)

    def rect(self, ref, pose=None):
        points, basis, reason = self.bodies.get(ref, (None, None, 'missing body geometry'))
        if points is None:
            return None, basis, reason
        fp = self.pcb.footprints[ref]
        x, y, rot = pose if pose is not None else (fp.x, fp.y, fp.rotation or 0.0)
        c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        pts = [(x+c*a+s*b, y-s*a+c*b) for a, b in points]
        return (min(p[0] for p in pts), min(p[1] for p in pts),
                max(p[0] for p in pts), max(p[1] for p in pts)), basis, ''

    def measure(self, ref, edge, pose=None):
        rect, basis, reason = self.rect(ref, pose)
        row = {'body_overhang_mm': None, 'body_setback_mm': None,
               'body_signed_position_mm': None, 'body_overhang_basis': basis,
               'overhang_mm': None, 'overhang_basis': basis, 'units': 'mm',
               'boundary_basis': 'rectangular Edge.Cuts centreline',
               'mating_face_basis': 'drawn envelope support line toward declared edge',
               'body_measured': False}
        reason = reason or self.boundary_reason
        if edge not in EDGES:
            reason = reason or 'no declared mating edge; a nearest-edge guess is not a requirement'
        if reason:
            return dict(row, body_unmeasured_reason=reason)
        b = self.bounds
        signed = {'west': b[0]-rect[0], 'east': rect[2]-b[2],
                  'north': b[1]-rect[1], 'south': rect[3]-b[3]}[edge]
        row.update(body_overhang_mm=max(0.0, signed), body_setback_mm=max(0.0, -signed),
                   body_signed_position_mm=signed, overhang_mm=max(0.0, signed),
                   body_measured=True, body_bounds_mm=list(rect))
        # A band licenses only the declared boundary, never a second edge.
        row['other_body_edge_overhang_mm'] = {
            e: max(0.0, v) for e, v in zip(EDGES,
                (b[0]-rect[0], rect[2]-b[2], b[1]-rect[1], rect[3]-b[3])) if e != edge}
        return row

    def inferred_edge(self, ref, pose=None):
        rect, _basis, reason = self.rect(ref, pose)
        if reason or self.boundary_reason:
            return None
        b = self.bounds
        gaps = [rect[0]-b[0], b[2]-rect[2], rect[1]-b[1], b[3]-rect[3]]
        outside = [e for e, g in zip(EDGES, gaps) if g < -EPS]
        if outside:
            return outside[0] if len(outside) == 1 else None
        nearest = min(abs(v) for v in gaps)
        choices = [e for e, g in zip(EDGES, gaps) if abs(abs(g)-nearest) <= EPS]
        return choices[0] if len(choices) == 1 else None


def state_measure(state, ref, edge, x=None, y=None):
    """Shared candidate measurement; never cache candidate poses."""
    if not hasattr(state, '_connector_geometry'):
        state._connector_geometry = ConnectorGeometry(state.pcb_data, state.pcb_file)
    part = state.parts[ref]
    return state._connector_geometry.measure(ref, edge,
        (part.x if x is None else x, part.y if y is None else y, part.rot))


def candidate_copper(state, ref, x, y):
    """All pad copper at a trial pose, through the inherited exact edge grader.

    No net filtering, centre-only predicate or body-band copper allowance.
    Copies retain shape approximations so unsupported geometry cannot certify.
    """
    from copy import copy
    from kicad_parser import _PAD_ORTHO_TOL, _resolve_pad_rect
    from .legality import grade_pad_edge_clearance
    source = state.pcb_data.footprints[ref]
    fp = copy(source)
    delta = state.parts[ref].rot - (source.rotation or 0.0)
    c, s = math.cos(math.radians(delta)), math.sin(math.radians(delta))
    def transform(a, b):
        a, b = a-source.x, b-source.y
        return x+c*a+s*b, y-s*a+c*b
    fp.pads = []
    for original in source.pads:
        pad = copy(original)
        pad.global_x, pad.global_y = transform(original.global_x, original.global_y)
        sx, sy = original.size_x, original.size_y
        if abs((original.rotation or 0.0) % 180 - 90) <= _PAD_ORTHO_TOL:
            sx, sy = sy, sx
        pad.rotation = (original.rotation or 0.0) + delta
        pad.size_x, pad.size_y, pad.rect_rotation = _resolve_pad_rect(sx, sy, pad.rotation)
        if getattr(original, 'polygons', None):
            pad.polygons = [[transform(a, b) for a, b in poly] for poly in original.polygons]
        fp.pads.append(pad)
    fp.x, fp.y, fp.rotation = x, y, state.parts[ref].rot
    pcb = copy(state.pcb_data)
    pcb.footprints = {ref: fp}
    return grade_pad_edge_clearance(pcb, state.board_edge_clearance, state.pcb_file)
