"""A connector's DRAWN body against a declared compass board edge (#961).

`rule_edge_connector` used to grade a declared `overhang_mm` band on
`rect_outside_amount(part.rect)`. That is a CLEARANCE SHORTFALL at the gate's
margin -- `margin - gap` inside the board, `overhang + margin` outside -- so a
part 0.15 mm inside the edge read 0.10 at margin 0.25 and satisfied a band
while `check_drc --check-pad-edge` failed the same geometry. Measured on
esp_prog's USB1 translated 1.45 mm west: the drawn body overhangs 1.45 mm and
the rule read 0.10 or 0.40 depending on the margin.

This module measures the quantity the band is about. The mating face is the
support line of the drawn envelope in the declared outward direction; the
signed position is positive outside the board, `overhang = max(0, signed)` and
`setback = max(0, -signed)`. It is a 2-D drawing contract, not a 3-D mating
simulation, and it takes no margin at all.

WHAT COUNTS AS A BODY. A closed convex polygonal envelope on the footprint's
own Fab layer -- or its own SilkS layer when it draws nothing on Fab; an
unusable Fab drawing does NOT fall back to SilkS -- drawn with `fp_line`, sharp-cornered
`fp_rect` and straight-sided `fp_poly`. Internal markings drawn with those
primitives are allowed, but every side of the convex hull must actually be
drawn. ANY other primitive on that layer makes the whole body UNMEASURED --
`fp_arc`, `fp_circle` (a pin-1 dot included), `fp_curve`, a rounded
`fp_rect`, an `fp_poly` carrying an arc -- as do open outlines, concave
envelopes and a part that draws no body. Text, pads and courtyards are never
body geometry. Vertices are transformed before taking extrema, so any
rotation of the supported primitives is exact.

WHAT COUNTS AS A BOUNDARY. A rectangular Edge.Cuts outline with no other cuts.
Anything else is unmeasured rather than approximated by the bounding box.

UNMEASURED IS A BASIS, NOT A VERDICT. `band_amount` hands every caller its
own legacy reading back, named `legacy_occupancy@margin=<m>`, when the body
cannot be measured. Nothing here refuses a pose, withholds a board or records
an abstention; a board this module cannot read grades exactly as it did before
#961, and says which currency it was graded in.
"""
from __future__ import annotations

import hashlib
import math
import os
import re
from collections import OrderedDict

from kicad_parser import find_matching_paren
from .parser import _footprint_blocks

EPS = 1e-6
EDGES = ('west', 'east', 'north', 'south')

#: Parsed board FILES, keyed by path AND content digest, so a board rewritten
#: in place between two reads is never answered from the old text. Body
#: envelopes are footprint-LOCAL, so a pose change never invalidates one.
_SOURCES: 'OrderedDict[tuple, _BoardSource]' = OrderedDict()
_SOURCES_LIMIT = 16


def _point(text, name):
    m = re.search(r'\(' + name + r'\s+([-+\d.eE]+)\s+([-+\d.eE]+)\)', text)
    if not m:
        raise ValueError('missing ' + name)
    p = tuple(float(v) for v in m.groups())
    if not all(math.isfinite(v) for v in p):
        raise ValueError('nonfinite body coordinate')
    return p


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _envelope(segments):
    """The convex hull of the drawn segments, refused unless every hull side
    is covered by drawn segments (a closed convex outline)."""
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
            if (abs(_cross(a, b, c)) / length > EPS
                    or abs(_cross(a, b, d)) / length > EPS):
                continue
            runs.append(sorted(((p[0] - a[0]) * (b[0] - a[0]) / length
                                + (p[1] - a[1]) * (b[1] - a[1]) / length)
                               for p in (c, d)))
        end = 0.0
        for lo, hi in sorted(runs):
            if lo > end + EPS:
                break
            end = max(end, hi)
        if end < length - EPS:
            raise ValueError('open or concave body envelope '
                             '(a convex hull side is not drawn)')
    return hull


def _parse_body(block, side):
    """`(hull, layer, reason)` for one footprint block on side 'F' or 'B'.

    A saved B-side footprint already stores mirrored local coordinates, so
    the side only selects WHICH layer is the part's own."""
    if block is None:
        return None, None, 'footprint block not found in the board file'
    layers = {}
    for m in re.finditer(r'\(fp_(\w+)\b', block):
        kind = m.group(1)
        if kind in ('text', 'text_box'):
            continue
        item = block[m.start():find_matching_paren(block, m.start())]
        lm = re.search(r'\(layer\s+"?([FB]\.(?:Fab|SilkS))"?\)', item)
        if lm:
            layers.setdefault(lm.group(1), []).append((kind, item))
    layer = next((side + suffix for suffix in ('.Fab', '.SilkS')
                  if side + suffix in layers), None)
    if layer is None:
        return None, None, 'no body drawing on the footprint face'
    try:
        segments = []
        for kind, item in layers[layer]:
            if kind == 'line':
                segments.append((_point(item, 'start'), _point(item, 'end')))
            elif kind == 'rect':
                rm = re.search(r'\(radius\s+([-+\d.eE]+)\)', item)
                if rm and float(rm.group(1)) > EPS:
                    # KiCad 10 rounded rectangle: its sharp corners would
                    # overstate the extent at any non-orthogonal rotation.
                    raise ValueError('rounded fp_rect corners are not measured')
                a, b = _point(item, 'start'), _point(item, 'end')
                pts = [a, (b[0], a[1]), b, (a[0], b[1])]
                segments.extend(zip(pts, pts[1:] + pts[:1]))
            elif kind == 'poly':
                if re.search(r'\(arc\b', item):
                    # An arc inside `pts` bulges past its chord; the chord
                    # polygon would understate the body.
                    raise ValueError('fp_poly with an arc segment is not '
                                     'measured')
                pts = [_point(x.group(), 'xy') for x in re.finditer(
                    r'\(xy\s+[-+\d.eE]+\s+[-+\d.eE]+\)', item)]
                segments.extend(zip(pts, pts[1:] + pts[:1]))
            else:
                raise ValueError('unsupported body primitive: fp_' + kind)
        return _envelope(segments), layer, ''
    except ValueError as exc:
        return None, layer, str(exc)


class _BoardSource:
    """What one board FILE says independently of any pose: its footprint
    blocks, and whether its Edge.Cuts is a plain rectangle."""

    def __init__(self, text, bounds, reason=''):
        self.blocks = {}
        self.envelopes = {}
        self.boundary_reason = reason
        if text is None:
            return
        from kicad_parser import _collect_edge_cuts_segments
        from .legality import _segments_cover_rectangle
        try:
            rectangular = bool(bounds) and _segments_cover_rectangle(
                _collect_edge_cuts_segments(text), bounds)
        except ValueError as exc:
            rectangular = False
            reason = 'board outline unreadable: ' + str(exc)
        if not rectangular:
            self.boundary_reason = reason or (
                'unsupported boundary: requires a rectangular Edge.Cuts '
                'outline without cutouts, curves or extra cuts')
        self.blocks = dict(_footprint_blocks(text))

    def envelope(self, ref, side):
        key = (ref, side)
        if key not in self.envelopes:
            self.envelopes[key] = _parse_body(self.blocks.get(ref), side)
        return self.envelopes[key]


def _source(path, bounds):
    if not path:
        return _BoardSource(None, bounds,
                            'no board file to read the drawn body from')
    try:
        with open(path, 'rb') as stream:
            raw = stream.read()
    except OSError as exc:
        return _BoardSource(None, bounds,
                            'body/boundary source unreadable: ' + str(exc))
    key = (os.path.normcase(os.path.abspath(path)),
           hashlib.blake2b(raw, digest_size=16).digest(),
           tuple(bounds) if bounds else None)
    hit = _SOURCES.get(key)
    if hit is None:
        try:
            text = raw.decode('utf-8')
        except UnicodeDecodeError as exc:
            hit = _BoardSource(None, bounds,
                               'body/boundary source unreadable: ' + str(exc))
        else:
            hit = _BoardSource(text, bounds)
        _SOURCES[key] = hit
        while len(_SOURCES) > _SOURCES_LIMIT:
            _SOURCES.popitem(last=False)
    else:
        _SOURCES.move_to_end(key)
    return hit


class ConnectorGeometry:
    """Drawn-body measurements for the footprints of one parsed board."""

    def __init__(self, pcb_data, path):
        self.pcb = pcb_data
        self.path = path
        self.bounds = getattr(pcb_data.board_info, 'board_bounds', None)
        self.source = _source(path, self.bounds)
        self.boundary_reason = self.source.boundary_reason

    def rect(self, ref, pose=None):
        """`(board_rect, layer, reason)` of the drawn envelope at `pose`
        (`(x, y, rotation)`; the parsed footprint's own pose when None)."""
        fp = self.pcb.footprints.get(ref)
        if fp is None:
            return None, None, 'part not on this board'
        from .legality import footprint_side
        points, layer, reason = self.source.envelope(ref, footprint_side(fp))
        if points is None:
            return None, layer, reason
        x, y, rot = pose if pose is not None else (fp.x, fp.y,
                                                    fp.rotation or 0.0)
        c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        pts = [(x + c * a + s * b, y - s * a + c * b) for a, b in points]
        return (min(p[0] for p in pts), min(p[1] for p in pts),
                max(p[0] for p in pts), max(p[1] for p in pts)), layer, ''

    def measure(self, ref, edge, pose=None):
        rect, layer, reason = self.rect(ref, pose)
        row = {'body_measured': False, 'body_layer': layer,
               'body_overhang_mm': None, 'body_setback_mm': None,
               'body_signed_position_mm': None, 'units': 'mm',
               'boundary_basis': 'rectangular Edge.Cuts centreline',
               'mating_face_basis': 'drawn envelope support line toward '
                                    'the declared edge'}
        reason = reason or self.boundary_reason
        if not reason and edge not in EDGES:
            reason = ('no declared mating edge; a nearest-edge guess is not '
                      'a requirement')
        if reason:
            row['body_unmeasured_reason'] = reason
            return row
        b = self.bounds
        outside = dict(zip(EDGES, (b[0] - rect[0], rect[2] - b[2],
                                   b[1] - rect[1], rect[3] - b[3])))
        signed = outside[edge]
        others = {e: max(0.0, v) for e, v in outside.items() if e != edge}
        row.update(body_measured=True,
                   body_overhang_mm=max(0.0, signed),
                   body_setback_mm=max(0.0, -signed),
                   body_signed_position_mm=signed,
                   body_bounds_mm=list(rect),
                   other_body_edge_overhang_mm=others,
                   # The body's overhang SUMMED over every side -- the form
                   # of the occupancy reading it replaces, at zero margin, so
                   # a corner part counts both sides against its band exactly
                   # as it did before #961. Equal to `body_overhang_mm`
                   # whenever the body crosses no second edge.
                   body_outside_mm=max(0.0, signed) + sum(others.values()))
        return row


def geometry_for(holder, pcb_data, pcb_file):
    """One `ConnectorGeometry` per holder (a grade context or a search state),
    built on first use -- no board without an edge connector pays for it."""
    geometry = getattr(holder, '_connector_geometry', None)
    if geometry is None or geometry.pcb is not pcb_data:
        geometry = ConnectorGeometry(pcb_data, pcb_file)
        holder._connector_geometry = geometry
    return geometry


def band_amount(geometry, ref, edge, legacy_amount, margin, pose=None):
    """`(amount, basis, row)`: the number a declared `overhang_mm` band is
    graded on, and the currency it is in.

    Drawn body measurable -> the body's overhang past the outline at zero
    margin, summed over the sides it crosses (`body_outside_mm`, the same
    form as the occupancy reading). Otherwise -> `legacy_amount`, exactly as
    the caller computed it, so every call site keeps its own pre-#961 reading
    (and its own rings and tolerance) wherever the body cannot be read."""
    row = geometry.measure(ref, edge, pose)
    if row['body_measured']:
        return row['body_outside_mm'], 'body:' + row['body_layer'], row
    return (float(legacy_amount),
            f'legacy_occupancy@margin={float(margin):g}', row)
