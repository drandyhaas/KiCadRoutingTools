"""Protected Kelvin sense branches: new copper joins them only at the join pad.

A buck regulator's VOS/divider trace, a load switch's CT-cap return: same net as
the high-current distribution, but load current must enter at ONE star pad (the
join), never ride the sense copper. The router routes a net by connecting its
pieces to whatever copper the net already has, so on a real board it attached
0.5 mm distribution branches straight onto a 0.2 mm VOS trace -- connected,
DRC-clean, wrong (#1279).

The declaration is the board's own: a KiCad group named `sense:<REF>.<PAD>`,
whose members are the sense tracks/arcs/vias and whose name is the join pad,

    (group "sense:C108.1" (uuid ...) (members "<track uuid>" "<via uuid>" ...))

The sense pads are the pads of the same net that member copper touches, the
join pad excepted. Contact is KiCad's: copper meets copper where the shapes
do -- a track crossing another in an X, a track across a pad off-centre, a
via only on the layers of its span -- and every pad is its real copper (a
round pad is a circle, not its bounding box). A member uuid that matches no
item is dropped, as KiCad drops it on load. There is no flag and no side
file: both parse paths
(`parse_kicad_pcb`, `build_pcb_data_from_board`) resolve the groups into
`PCBData.sense_branches`, so every step of a chain and every leg that parses
the board sees them. Nothing is moved to another net.

The rule, for the branch's own net (every other net already sees the branch as
foreign copper):
- obstacles: `build_base_obstacle_map` stamps member copper (from where its
  copper leaves the join pad) and sense pads as foreign copper at the net's
  clearance, holding to it only the new copper that sticks out of the join
  pad (`stamp_base_obstacles`, `_still_blocked`; the pad's own cell is always
  free, so a pad narrower than the track is landed at its centre), so every
  map derived from a base -- per-net via rungs included -- carries it;
- terminal gate: `_neck_terminal_grazes` measures the same copper with the
  same off-pad measure (`terminal_sense_dist`);
- terminals: the endpoint derivations never offer member copper or a sense pad
  as a place to attach (`connectivity.routable`, net_rescue's join points), so
  the net's other pads route to the join pad;
- immutability: member copper carries `held_by` (kicad_parser.
  is_immutable_copper), so no rip, prune, smoothing, layer-switch or move
  pass rewrites or removes it, even when the branch is refused. It is not a
  KiCad lock and is never written: the net's routes of this run stay
  rippable, and only whole-net strips of its pre-existing copper refuse it
  (protected_nets.protection_map, reason `sense:<REF>.<PAD>`).

A branch is reported and left unprotected (JSON_SUMMARY.sense_branches carries
the reason) when the group name is not `sense:REF.PAD`, the join pad is
missing or has no net, a member is not a track or via on the board, member
copper is on another net, member copper does not touch the join pad, a sense
pad or member is not joined to the join pad by member copper yet, other copper
of the net already touches the branch off the join pad (the input taps it), or
a pour of the same net can fill over the branch off the join pad (KiCad's fill
floods over same-net copper, so the pour already taps it; the zone's cut-outs
and copper-pour keep-outs are not fill, so a pour cut around the trace does
not).
KiCad DRC cannot see topology, so check the written board in every case.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

#: Contact slack in mm.
_EPS = 0.01

def _in_pad(pad, x: float, y: float, grow: float) -> bool:
    """True when (x, y) lies on the pad's REAL copper grown by `grow` mm
    (`grow` >= 0): check_drc.point_to_pad_distance, which holds custom
    polygons, round corners and the residual tilt `rect_rotation`. The pad
    rectangle was wrong for a round, oval or roundrect pad, whose bounding-box
    corners are not copper: a member ending 0.24 mm off a round join pad's
    copper read as joined to it, and the cut-out let new copper land on
    member copper beyond the circle."""
    from check_drc import point_to_pad_distance
    return point_to_pad_distance(x, y, pad) <= grow


def _pad_sd(pad, xs, ys):
    """Signed distance (mm) from each point to the pad's real copper:
    negative inside (minus the depth), positive outside. The vectorised,
    signed twin of check_drc.point_to_pad_distance, on the same shape
    (check_drc._pad_copper_core: the pad is its inner polygon grown by its
    corner radius -- a point for a circle, a segment for an oval)."""
    from check_drc import _pad_copper_core
    from check_connected import points_in_polygon_mask
    xs = np.atleast_1d(np.asarray(xs, dtype=np.float64))
    ys = np.atleast_1d(np.asarray(ys, dtype=np.float64))
    if not getattr(pad, 'polygons', None):
        # Closed form for a rect / roundrect / circle / oval pad: the rounded
        # box (inner half-extents grown by the corner radius) in the pad's
        # frame -- the same shape as walking its core polygon, far cheaper.
        hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
        if pad.shape in ('circle', 'oval'):
            r = min(hx, hy)
        elif pad.shape == 'roundrect':
            r = getattr(pad, 'roundrect_rratio', 0.0) * min(pad.size_x, pad.size_y)
        else:
            r = 0.0
        r = min(r, hx, hy)
        dx, dy = xs - pad.global_x, ys - pad.global_y
        rr = getattr(pad, 'rect_rotation', 0.0) or 0.0
        if rr:
            c, s = math.cos(math.radians(rr)), math.sin(math.radians(rr))
            dx, dy = dx * c + dy * s, -dx * s + dy * c
        qx, qy = np.abs(dx) - (hx - r), np.abs(dy) - (hy - r)
        return (np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
                + np.minimum(np.maximum(qx, qy), 0.0) - r)
    polys, radius = _pad_copper_core(pad)
    best = np.full(xs.shape, np.inf)
    for poly in polys:
        if not poly:
            continue
        P = np.asarray(poly, dtype=np.float64)
        Q = np.roll(P, -1, axis=0)
        x1, y1 = P[:, 0][None, :], P[:, 1][None, :]
        dx, dy = (Q[:, 0] - P[:, 0])[None, :], (Q[:, 1] - P[:, 1])[None, :]
        l2 = dx * dx + dy * dy
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.clip(((xs[:, None] - x1) * dx + (ys[:, None] - y1) * dy) / l2, 0.0, 1.0)
        t = np.where(l2 < 1e-18, 0.0, t)
        d = np.sqrt(((xs[:, None] - (x1 + t * dx)) ** 2
                     + (ys[:, None] - (y1 + t * dy)) ** 2).min(axis=1))
        inside = points_in_polygon_mask(xs, ys, poly) if len(P) >= 3 else False
        best = np.minimum(best, np.where(inside, -d, d))
    return best - radius


def _inradius(pad) -> float:
    """How deep the pad's centre lies in its copper (min(size)/2 for a rect,
    circle, oval or roundrect pad; the anchor's depth for a custom one)."""
    return max(0.0, -float(_pad_sd(pad, [pad.global_x], [pad.global_y])[0]))


def _dist_seg(px, py, ax, ay, bx, by) -> float:
    """Point to segment-axis distance."""
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _pad_on(pad, layer: str) -> bool:
    return layer in pad.layers or '*.Cu' in pad.layers


def _via_pad_layers(via, pad) -> list:
    """The copper layers `via` and `pad` share: a blind, buried or micro via
    has copper on its span only (geometry_utils.via_on_layer), so it meets an
    SMD pad on another layer nowhere."""
    from geometry_utils import via_on_layer
    if '*.Cu' in pad.layers:
        return [L for L in (getattr(via, 'layers', None) or ()) if L.endswith('.Cu')] or ['F.Cu']
    return [L for L in pad.layers if L.endswith('.Cu') and via_on_layer(via, L)]


def _vias_share_layer(a, b) -> bool:
    """Two vias' spans overlap: one has an end layer the other covers."""
    from geometry_utils import via_on_layer
    ends = [L for L in list(getattr(a, 'layers', None) or ()) + list(getattr(b, 'layers', None) or ())
            if L.endswith('.Cu')]
    return not ends or any(via_on_layer(a, L) and via_on_layer(b, L) for L in ends)


def _via_meets_pad(via, pad, grow: float = _EPS) -> bool:
    """The via's barrel reaches the pad's real copper on a layer both carry
    (connectivity.endpoint_reaches_pad, check_connected's via-in-pad rule)."""
    from connectivity import endpoint_reaches_pad
    layers = _via_pad_layers(via, pad)
    return bool(layers) and bool(endpoint_reaches_pad(via.x, via.y, via.size / 2 + grow,
                                                      layers, pad))


def _via_on_join(via, join) -> bool:
    """The via's centre is on the join pad's copper on a layer both carry --
    a via there is the join itself, not member copper off it."""
    return bool(_via_pad_layers(via, join)) and _in_pad(join, via.x, via.y, 0.0)


def _pad_reach(pad) -> float:
    """Radius about the pad's centre that holds all its copper (a custom pad's
    polygons may reach past its anchor `size`) -- a prefilter only."""
    r = math.hypot(pad.size_x, pad.size_y) / 2
    for poly in getattr(pad, 'polygons', None) or ():
        for x, y in poly:
            r = max(r, math.hypot(x - pad.global_x, y - pad.global_y))
    return r


def _seg_meets_pad(seg, pad, grow: float = _EPS):
    """(touches, point): whether the track's copper reaches the pad's REAL
    copper (custom polygons, round corners and tilt held, within `grow`) on
    the track's layer, anywhere along it -- a track through a pad touches it
    with both ends outside -- and the point on the track nearest the pad
    (where it first meets the copper from its start, when it enters it).

    Exact, on check_connected's own measure: check_drc.segment_pad_gap, which
    track_reaches_pad asks. The 0.05 mm samples of check_pad_segment_overlap
    it replaced stood either side of a narrow corner graze and missed it."""
    if not _pad_on(pad, seg.layer) or _dist_seg(
            pad.global_x, pad.global_y, seg.start_x, seg.start_y, seg.end_x,
            seg.end_y) > _pad_reach(pad) + seg.width / 2 + grow:
        return False, None
    from check_drc import segment_pad_gap
    if segment_pad_gap(seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                       pad) >= seg.width / 2 + grow:
        return False, None
    return True, _track_point_nearest_pad(seg, pad)


def _track_point_nearest_pad(seg, pad) -> Tuple[float, float]:
    """The point on the track's centreline nearest the pad's copper, the first
    from its start where it lies on the copper. The gap to a polygon's edges
    is least at a centreline end, at a polygon vertex's foot on the
    centreline, or where the centreline crosses an edge, so those are the
    only places to look (a round pad's core grown by its radius has the same
    nearest points)."""
    from check_drc import _pad_copper_core
    x1, y1 = seg.start_x, seg.start_y
    dx, dy = seg.end_x - x1, seg.end_y - y1
    l2 = dx * dx + dy * dy
    ts = {0.0, 1.0}
    for poly in _pad_copper_core(pad)[0]:
        n = len(poly)
        for i in range(n):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % n]
            if l2 > 0:
                ts.add(min(1.0, max(0.0, ((ax - x1) * dx + (ay - y1) * dy) / l2)))
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            if den:
                t = ((ax - x1) * ey - (ay - y1) * ex) / den
                u = ((ax - x1) * dy - (ay - y1) * dx) / den
                if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
                    ts.add(t)
    t = np.array(sorted(ts))
    gap = np.maximum(_pad_sd(pad, x1 + dx * t, y1 + dy * t), 0.0)
    k = int(np.argmin(gap))         # the first of equal gaps: nearest the start
    return float(x1 + dx * t[k]), float(y1 + dy * t[k])


#: Sampling step (mm) along a track when locating where it meets other copper.
_CONTACT_STEP = 0.02


def _seg_seg_contacts(a, b):
    """(xs, ys): where track `a`'s copper meets track `b`'s, sampled every
    _CONTACT_STEP along `a` -- for each sample within reach of `b`'s axis, the
    point between the two centrelines at `a`'s share of the two half-widths,
    which lies in their overlap. An end landing on `b`, a T and an X crossing
    all give their contact this way; an end alone does not say WHERE two
    tracks meet."""
    ra, rb = a.width / 2.0, b.width / 2.0
    ex, ey = a.end_x - a.start_x, a.end_y - a.start_y
    n = max(2, int(math.hypot(ex, ey) / _CONTACT_STEP) + 2)
    t = np.linspace(0.0, 1.0, n)
    xs, ys = a.start_x + ex * t, a.start_y + ey * t
    fx, fy = b.end_x - b.start_x, b.end_y - b.start_y
    l2 = fx * fx + fy * fy
    u = (np.clip(((xs - b.start_x) * fx + (ys - b.start_y) * fy) / l2, 0.0, 1.0)
         if l2 > 0 else np.zeros_like(xs))
    px, py = b.start_x + u * fx, b.start_y + u * fy
    hit = np.hypot(px - xs, py - ys) <= ra + rb + _EPS
    k = ra / (ra + rb) if ra + rb > 0 else 0.5
    return xs[hit] + (px[hit] - xs[hit]) * k, ys[hit] + (py[hit] - ys[hit]) * k


def _touch(a, b, join) -> bool:
    """Copper contact between two items of one net anywhere off the join pad,
    judged WHERE the copper meets: off the pad's real copper by more than
    copper of the items' width centred on a pad narrower than it must
    overhang (_overhang_allowance; the contact slack otherwise). Judging an
    end instead flagged a 1 mm track that ends 0.025 mm past a 0.8 mm join pad
    and meets the member on the pad as a tap (and the run as tapped_by_run).
    Items are ('pad', Pad) | ('seg', Segment) | ('via', Via)."""
    order = {'seg': 0, 'via': 1, 'pad': 2}
    if order[a[0]] > order[b[0]]:
        a, b = b, a
    (ka, ia), (kb, ib) = a, b

    def off(x, y, half):
        return not _in_pad(join, x, y, _overhang_allowance(join, half))

    if ka == 'seg' and kb == 'seg':
        if ia.layer != ib.layer:
            return False
        reach = ia.width / 2 + ib.width / 2 + _EPS
        if (min(ia.start_x, ia.end_x) > max(ib.start_x, ib.end_x) + reach
                or max(ia.start_x, ia.end_x) < min(ib.start_x, ib.end_x) - reach
                or min(ia.start_y, ia.end_y) > max(ib.start_y, ib.end_y) + reach
                or max(ia.start_y, ia.end_y) < min(ib.start_y, ib.end_y) - reach):
            return False
        # Two tracks crossing in an X have no end near the other, and KiCad
        # joins them all the same (its connectivity collides the shapes).
        half = max(ia.width, ib.width) / 2
        return any(off(x, y, half) for x, y in zip(*_seg_seg_contacts(ia, ib)))
    # A via has copper on its span only: a blind F.Cu-In1.Cu via never meets
    # B.Cu copper above or below it.
    if ka == 'via' and kb == 'via':
        ra, rb = ia.size / 2, ib.size / 2
        d = math.hypot(ia.x - ib.x, ia.y - ib.y)
        k = ra / (ra + rb) if ra + rb > 0 else 0.5
        return (_vias_share_layer(ia, ib) and d <= ra + rb
                and off(ia.x + (ib.x - ia.x) * k, ia.y + (ib.y - ia.y) * k, max(ra, rb)))
    if ka == 'seg' and kb == 'via':
        from geometry_utils import closest_point_on_segment, via_on_layer
        if not via_on_layer(ib, ia.layer):
            return False
        px, py = closest_point_on_segment(ib.x, ib.y, ia.start_x, ia.start_y, ia.end_x, ia.end_y)
        rv, rs = ib.size / 2, ia.width / 2
        if math.hypot(px - ib.x, py - ib.y) > rv + rs + _EPS:
            return False
        k = rv / (rv + rs) if rv + rs > 0 else 0.5
        return off(ib.x + (px - ib.x) * k, ib.y + (py - ib.y) * k, max(rv, rs))
    if ka == 'seg' and kb == 'pad':
        # Anywhere along the track, not only at its ends or the pad's centre:
        # a sense trace running across a same-net pad off-centre is joined to
        # it (KiCad's connectivity collides the shapes).
        hit, pt = _seg_meets_pad(ia, ib)
        return hit and off(*pt, ia.width / 2)
    if ka == 'via' and kb == 'pad':
        return off(ia.x, ia.y, ia.size / 2) and _via_meets_pad(ia, ib)
    return False


def _touches_join(item, join) -> bool:
    """True when the item has copper on the join pad -- for a track anywhere
    along it: a member running across the join pad with both ends outside it
    is joined to it."""
    kind, it = item
    if kind == 'seg':
        return _seg_meets_pad(it, join)[0]
    if kind == 'via':
        return _via_meets_pad(it, join)
    return False


def _point_in_poly(x: float, y: float, poly) -> bool:
    """Ray-casting point-in-polygon."""
    inside, j = False, len(poly) - 1
    for i in range(len(poly)):
        (xi, yi), (xj, yj) = poly[i], poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _pour_keepouts(pcb_data, layer: str) -> list:
    """The rule areas forbidding copper pour on `layer`, as rings (outline
    first, then its holes): KiCad's filler subtracts them from every zone
    (the layer reading of plane_fill_model's #477 stamp)."""
    out = []
    for ko in (getattr(pcb_data.board_info, 'keepouts', None) or ()):
        if ko.get('copper_pour_allowed', True):
            continue
        kls = ko.get('layers') or set()
        if kls and layer not in kls and '*.Cu' not in kls and not (
                layer in ('F.Cu', 'B.Cu') and ({'F&B.Cu', 'F&B'} & kls)):
            continue
        rings = [r for r in [ko.get('polygon') or []] + list(ko.get('holes') or ())
                 if len(r) >= 3]
        if rings:
            out.append(rings)
    return out


def _zone_tap(pcb_data, items, join) -> Optional[str]:
    """Why a pour of the branch's own net already taps its sense copper, or
    None. A zone floods over same-net copper, so sense copper inside (or
    touching) the area a same-net pour may fill on its layer is joined to
    that pour -- and through it to every load the pour feeds -- in the board
    KiCad fills, whatever the router keeps off it. That area is the zone's
    outline minus its cut-outs (Zone.holes) and minus every rule area
    forbidding pour on the layer: a pour cut around the sense trace does not
    tap it. A sense pad is judged by its copper, not its centre: a pour
    overlapping a strip of it is a tap. Contact on the join pad itself is
    the join, not a tap."""
    from kicad_parser import pour_zones
    from connectivity import via_copper_layers
    from check_drc import _pad_perimeter_points
    net = join.net_id
    zones = [z for z in pour_zones(getattr(pcb_data, 'zones', None))
             if z.net_id == net and len(z.polygon or ()) >= 3]
    if not zones:
        return None
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or ())
    for z in zones:
        cuts = [h for h in (getattr(z, 'holes', None) or ()) if len(h) >= 3]
        kos = _pour_keepouts(pcb_data, z.layer)

        def in_ko(x, y, rings):
            return _point_in_poly(x, y, rings[0]) and not any(
                _point_in_poly(x, y, h) for h in rings[1:])

        def in_fill(x, y):
            return (_point_in_poly(x, y, z.polygon)
                    and not any(_point_in_poly(x, y, h) for h in cuts)
                    and not any(in_ko(x, y, rings) for rings in kos))

        rings = [z.polygon] + cuts + [r for k in kos for r in k]
        edges = [(r[i], r[(i + 1) % len(r)]) for r in rings for i in range(len(r))]
        for kind, it in items:
            if kind == 'seg':
                if it.layer != z.layer:
                    continue
                n = max(1, int(math.ceil(math.hypot(it.end_x - it.start_x,
                                                    it.end_y - it.start_y) / 0.1)))
                pts = [(it.start_x + (it.end_x - it.start_x) * k / n,
                        it.start_y + (it.end_y - it.start_y) * k / n) for k in range(n + 1)]
                r, what = it.width / 2, f"its {it.layer} track"
            elif kind == 'via':
                if z.layer not in via_copper_layers(it, copper):
                    continue
                pts, r, what = [(it.x, it.y)], it.size / 2, "its via"
            else:
                if not _pad_on(it, z.layer):
                    continue
                pts = [(it.global_x, it.global_y)] + list(_pad_perimeter_points(it))
                r = 0.0
                what = f"sense pad {it.component_ref}.{it.pad_number}"
            hit = None
            for x, y in pts:
                if _in_pad(join, x, y, r + _EPS):
                    continue
                if in_fill(x, y) or any(
                        _dist_seg(x, y, a[0], a[1], b[0], b[1]) <= r + _EPS for a, b in edges):
                    hit = (x, y)
                    break
            if hit is None and kind == 'pad':
                # A pour corner poking into the pad between perimeter samples.
                hit = next(((x, y) for x, y in z.polygon
                            if _in_pad(it, x, y, 0.0) and not _in_pad(join, x, y, _EPS)), None)
            if hit is not None:
                return (f"a {z.net_name or net} zone on {z.layer} pours over {what} "
                        f"at ({hit[0]:.2f}, {hit[1]:.2f}) -- its fill already joins the sense "
                        f"copper to the net")
    return None


def _joined_to(items, join) -> Set[int]:
    """Indices of `items` (a sense component) that copper joins to the join
    pad: the items touching it, and everything they reach off the pad."""
    seen: Set[int] = {i for i, it in enumerate(items) if _touches_join(it, join)}
    queue = list(seen)
    while queue:
        i = queue.pop()
        for j, other in enumerate(items):
            if j not in seen and _touch(items[i], other, join):
                seen.add(j)
                queue.append(j)
    return seen


# ---------------------------------------------------------------------------
# The declaration: a KiCad group named `sense:<REF>.<PAD>`.
# ---------------------------------------------------------------------------

#: A group whose name starts with this declares a protected sense branch; the
#: rest of the name is the JOIN (star) pad, `REF.PAD`.
GROUP_PREFIX = 'sense:'


class SenseBranch:
    """One `sense:<REF>.<PAD>` group, resolved against a parsed board.

    `segments` / `vias` are the member copper, `sense_pads` the pads of the
    same net that member copper touches (the join pad excepted), `join` the
    join Pad. `reason` is None for a protected branch, else why it is not.
    `dangling_uuids` are member uuids the file lists that match no item on
    the board: KiCad drops them on load, so they are not members (`warnings`
    names them)."""

    def __init__(self, name: str, group_uuid: str, member_uuids: List[str]):
        self.name = name
        self.group_uuid = group_uuid
        self.member_uuids = sorted(member_uuids)
        self.dangling_uuids = sorted(getattr(member_uuids, 'dangling', None) or ())
        self.join_key: Optional[Tuple[str, str]] = None
        self.join = None
        self.net_id = 0
        self.segments: list = []
        self.vias: list = []
        self.sense_pads: list = []
        self.reason: Optional[str] = None

    @property
    def protected(self) -> bool:
        return self.reason is None

    def join_label(self) -> str:
        return f"{self.join_key[0]}.{self.join_key[1]}" if self.join_key else '?'

    @property
    def warnings(self) -> List[str]:
        """What the declaration says that the board does not hold."""
        if not self.dangling_uuids:
            return []
        n = len(self.dangling_uuids)
        return [f"member uuid(s) {', '.join(self.dangling_uuids[:4])}"
                f"{' ...' if n > 4 else ''} match no item on the board; KiCad drops "
                f"{'it' if n == 1 else 'them'} from the group on load, so "
                f"{'it is' if n == 1 else 'they are'} ignored"]

    def summary(self, pcb_data=None) -> dict:
        """The JSON_SUMMARY.sense_branches entry."""
        net = ''
        if pcb_data is not None and self.net_id in getattr(pcb_data, 'nets', {}):
            net = pcb_data.nets[self.net_id].name
        out = {'group': self.name, 'join': self.join_label(), 'net': net,
               'protected': self.protected,
               'segments': len(self.segments), 'vias': len(self.vias),
               'sense_pads': sorted(f"{p.component_ref}.{p.pad_number}"
                                    for p in self.sense_pads)}
        if self.reason is not None:
            out['reason'] = self.reason
        if self.warnings:
            out['warnings'] = self.warnings
        return out


def branch_signature(b: SenseBranch) -> tuple:
    """Order-free identity of a resolved branch, for comparing parse paths."""
    return (b.name, b.join_label(), tuple(b.member_uuids), b.reason or '',
            tuple(sorted(f"{p.component_ref}.{p.pad_number}" for p in b.sense_pads)),
            len(b.segments), len(b.vias))


_UUID_RE = r'"?([0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12})"?'


class _Members(list):
    """A group's member uuids that match an item, carrying the ones that do
    not as `dangling` (SenseBranch.dangling_uuids)."""
    dangling: List[str] = []


def sense_groups_from_text(content: str) -> List[Tuple[str, str, List[str]]]:
    """[(group name, group uuid, [member uuid, ...])] for every
    `(group "sense:..." ...)` block of a board file. The group's own
    `(uuid ...)` is its identity, not a member (read `(members ...)` only).

    A member uuid that matches no item's `(uuid ...)` / `(tstamp ...)` /
    `(id ...)` anywhere in the file is left out, as KiCad leaves it out of the
    group on load (pcbnew 9.0.6 and 10.0.3: GetItems() omits it), so both
    parse paths see the same members; it rides on the list as `dangling`."""
    import re
    from kicad_parser import find_matching_paren
    out = []
    if GROUP_PREFIX not in content:
        return out
    known = None
    for m in re.finditer(r'\(group\s+"(' + re.escape(GROUP_PREFIX) + r'[^"]*)"', content):
        block = content[m.start():find_matching_paren(content, m.start())]
        u = re.search(r'\((?:uuid|id)\s+' + _UUID_RE, block)
        mem = re.search(r'\(members\b', block)
        found = (re.findall(_UUID_RE, block[mem.start():find_matching_paren(block, mem.start())])
                 if mem else [])
        if found and known is None:
            known = {k.lower() for k in re.findall(
                r'\((?:uuid|tstamp|id)\s+' + _UUID_RE + r'\s*\)', content)}
        members = _Members(x for x in found if x.lower() in known)
        members.dangling = [x for x in found if x.lower() not in known]
        out.append((m.group(1), u.group(1) if u else '', members))
    return out


def sense_groups_from_board(board) -> List[Tuple[str, str, List[str]]]:
    """`sense_groups_from_text` for a live pcbnew BOARD (board.Groups())."""
    out = []
    try:
        groups = list(board.Groups())
    except Exception as e:  # noqa: BLE001 -- say so rather than protect nothing silently
        print(f"  WARNING: could not read the board's groups from pcbnew "
              f"({type(e).__name__}: {e}); sense branches (#1279) are not protected")
        return out
    for g in groups:
        name = str(g.GetName())
        if not name.startswith(GROUP_PREFIX):
            continue
        members = []
        for it in g.GetItems():
            try:
                members.append(str(it.m_Uuid.AsString()))
            except Exception:  # noqa: BLE001
                members.append('?')
        try:
            guuid = str(g.m_Uuid.AsString())
        except Exception:  # noqa: BLE001
            guuid = ''
        out.append((name, guuid, members))
    return out


def _item_label(kind: str, it) -> str:
    if kind == 'seg':
        return (f"track ({it.start_x:g}, {it.start_y:g})-({it.end_x:g}, {it.end_y:g}) "
                f"on {it.layer}")
    if kind == 'via':
        return f"via at ({it.x:g}, {it.y:g})"
    return f"pad {it.component_ref}.{it.pad_number}"


def _resolve_one(pcb_data, b: SenseBranch, seg_by_uuid, via_by_uuid,
                 copper: str = "input copper") -> Optional[str]:
    """Fill `b` from the board; return why it cannot be protected, or None.
    `copper` names the net's other copper in a tap refusal."""
    ref, dot, num = b.name[len(GROUP_PREFIX):].rpartition('.')
    if not dot or not ref or not num:
        return (f"group name {b.name!r} does not name the join pad as "
                f"{GROUP_PREFIX}REF.PAD")
    b.join_key = (ref, num)
    fp = pcb_data.footprints.get(ref)
    join = next((p for p in (fp.pads if fp is not None else ()) if p.pad_number == num), None)
    if join is None:
        return f"join pad {ref}.{num} is not on the board"
    b.join = join
    if not join.net_id:
        return f"join pad {ref}.{num} has no net"
    b.net_id = join.net_id
    if not b.member_uuids:
        return "the group has no member tracks or vias"
    unknown = []
    for u in b.member_uuids:
        segs, via = seg_by_uuid.get(u), via_by_uuid.get(u)
        if segs:
            b.segments.extend(segs)
        elif via is not None:
            b.vias.append(via)
        else:
            unknown.append(u)
    if unknown:
        return (f"member(s) {', '.join(unknown[:4])}{' ...' if len(unknown) > 4 else ''} "
                f"are not tracks or vias on the board")
    items = [('seg', s) for s in b.segments] + [('via', v) for v in b.vias]
    off = sorted({it.net_id for _k, it in items if it.net_id != join.net_id})
    if off:
        names = [pcb_data.nets[n].name if n in pcb_data.nets else str(n) for n in off]
        return f"member copper is on {', '.join(names)}, not the join pad's net"
    # Sense pads: every pad of the net the member copper touches off the join.
    b.sense_pads = [p for p in pcb_data.pads_by_net.get(join.net_id, ())
                    if p is not join and any(_touch(it, ('pad', p), join) for it in items)]
    allitems = items + [('pad', p) for p in b.sense_pads]
    joined = _joined_to(allitems, join)
    if not joined:
        return f"member copper does not touch the join pad {ref}.{num}"
    loose = [_item_label(*allitems[i]) for i in range(len(allitems)) if i not in joined]
    if loose:
        return (f"{', '.join(loose[:3])}{' ...' if len(loose) > 3 else ''} not joined to "
                f"{ref}.{num} by member copper -- route the branch first")
    # Input already taps it: other copper of the net touching the branch off
    # the join pad would carry load current onto the sense copper.
    ids = {id(it) for _k, it in allitems}
    others = ([('seg', s) for s in pcb_data.segments
               if s.net_id == join.net_id and id(s) not in ids]
              + [('via', v) for v in pcb_data.vias
                 if v.net_id == join.net_id and id(v) not in ids])
    taps = [_item_label(*o) for o in others if any(_touch(o, it, join) for it in allitems)]
    if taps:
        return (f"{copper} already taps it ({', '.join(taps[:3])}"
                f"{' ...' if len(taps) > 3 else ''})")
    return _zone_tap(pcb_data, allitems, join)


def _copper_by_uuid(pcb_data):
    """({uuid: [Segment, ...]}, {uuid: Via}) over the board's tracks and
    vias (an arc is several Segments under one uuid; graphics excluded)."""
    seg_by_uuid: Dict[str, list] = {}
    for s in pcb_data.segments:
        if s.uuid and not getattr(s, 'graphic', False):
            seg_by_uuid.setdefault(s.uuid, []).append(s)
    return seg_by_uuid, {v.uuid: v for v in pcb_data.vias if v.uuid}


def resolve_sense_branches(pcb_data, raw) -> List[SenseBranch]:
    """Resolve `sense_groups_from_*` output against a parsed board. Called by
    BOTH parse paths, so every step of a chain and every leg that parses the
    file sees the same branches. A protected branch's member copper and sense
    pads are tagged `sense = <group name>`."""
    out: List[SenseBranch] = []
    if not raw:
        return out
    seg_by_uuid, via_by_uuid = _copper_by_uuid(pcb_data)
    for name, guuid, members in raw:
        b = SenseBranch(name, guuid, members)
        b.reason = _resolve_one(pcb_data, b, seg_by_uuid, via_by_uuid)
        out.append(b)
    for b in out:
        # The declared copper is held immutable whenever it resolves on the
        # join pad's net, protected or refused: a refused branch is routed
        # like undeclared copper, but no pass may rewrite or remove it -- a
        # tapped branch used to lose its in-memory lock and the late collapse
        # deleted the sense trace as redundant input copper. `held_by`, not
        # `locked`: nothing writes it to the board, and only the net's
        # pre-existing copper is spared from whole-net strips.
        if b.join is not None and b.net_id:
            for it in b.segments + b.vias:
                if it.net_id == b.net_id:
                    it.held_by = b.name
        if b.protected:
            for it in b.segments + b.vias + b.sense_pads:
                it.sense = b.name
    return out


def protected_branches(pcb_data, net_id: Optional[int] = None) -> List[SenseBranch]:
    """The board's protected branches (of `net_id` when given)."""
    return [b for b in (getattr(pcb_data, 'sense_branches', None) or ())
            if b.protected and (net_id is None or b.net_id == net_id)]


def report(pcb_data) -> None:
    """Print one line per declared branch: protected, or why not."""
    branches = getattr(pcb_data, 'sense_branches', None) or []
    if not branches:
        return
    print(f"Sense branches (#1279): {len(branches)} declared by `{GROUP_PREFIX}` group(s)")
    for b in branches:
        if b.protected:
            pads = ', '.join(sorted(f"{p.component_ref}.{p.pad_number}" for p in b.sense_pads))
            print(f"  {b.name}: protected {len(b.segments)} track(s), {len(b.vias)} via(s), "
                  f"sense pad(s) {pads or 'none'}; new copper joins only at {b.join_label()}")
        else:
            print(f"  {b.name}: NOT protected -- {b.reason}")
        for w in b.warnings:
            print(f"  {b.name}: WARNING -- {w}")


def shipped_reason(pcb_data, b: SenseBranch) -> Optional[str]:
    """Why branch `b`, protected at the start of the run, no longer resolves
    as protected against the board's copper NOW (the run tapped it, or a
    member is gone), or None. Resolves a fresh copy: nothing is tagged."""
    fresh = SenseBranch(b.name, b.group_uuid, b.member_uuids)
    seg_by_uuid, via_by_uuid = _copper_by_uuid(pcb_data)
    return _resolve_one(pcb_data, fresh, seg_by_uuid, via_by_uuid,
                        copper="this run's copper")


def summary_entries(pcb_data) -> List[dict]:
    """JSON_SUMMARY.sense_branches: one entry per declared branch, as the
    input resolved it, plus `tapped_by_run` (with a printed WARNING) for a
    branch the input protected and the routed copper no longer does -- the
    run's verdict on the board it ships, not the input's."""
    out = []
    for b in (getattr(pcb_data, 'sense_branches', None) or []):
        entry = b.summary(pcb_data)
        if b.protected:
            why = shipped_reason(pcb_data, b)
            if why:
                entry['tapped_by_run'] = why
                print(f"  WARNING: sense branch {b.name} was protected at the "
                      f"start of this run but the routed copper breaks it: {why}")
        out.append(entry)
    return out


# ---------------------------------------------------------------------------
# The rule: member copper and sense pads are FOREIGN to their own net, except
# on the join pad.
# ---------------------------------------------------------------------------

def _overhang_allowance(join, half: float) -> float:
    """How far copper of half-width `half` may reach past the join pad's
    copper and still be ON the join: the contact slack, or -- on a pad
    narrower than the copper -- the overhang a track or via of that width
    centred on the pad cannot avoid."""
    return max(_EPS, half - _inradius(join))


def _is_plain_rect(pad) -> bool:
    """A pad whose copper is exactly its (tilted) size_x x size_y rectangle."""
    if getattr(pad, 'polygons', None):
        return False
    if pad.shape in ('circle', 'oval'):
        return False
    return not (pad.shape == 'roundrect' and getattr(pad, 'roundrect_rratio', 0.0) > 0)


#: Sampling step (mm) along a track when its on-pad span is not closed-form.
_CLIP_STEP = 0.005


def _on_pad_spans(seg, pad, depth: float = 0.0) -> list:
    """[(t0, t1), ...] of `seg`'s centreline lying `depth` mm or deeper inside
    the pad's real copper (t along start -> end). A plain rectangle clips in
    closed form (Liang-Barsky in the pad frame); a round, oval, roundrect or
    custom pad is sampled every _CLIP_STEP along the part of the track near
    the pad and each crossing bisected on _pad_sd."""
    if _is_plain_rect(pad):
        from routing_utils import _to_pad_frame
        ax, ay = _to_pad_frame(seg.start_x, seg.start_y, pad)
        bx, by = _to_pad_frame(seg.end_x, seg.end_y, pad)
        hx, hy = pad.size_x / 2.0 - depth, pad.size_y / 2.0 - depth
        if hx < 0 or hy < 0:
            return []
        t0, t1 = 0.0, 1.0
        dx, dy = bx - ax, by - ay
        for p, q in ((-dx, ax + hx), (dx, hx - ax), (-dy, ay + hy), (dy, hy - ay)):
            if abs(p) < 1e-12:
                if q < 0:
                    return []               # parallel and outside: no overlap
                continue
            r = q / p
            if p < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
        return [(t0, t1)] if t0 < t1 else []
    ex, ey = seg.end_x - seg.start_x, seg.end_y - seg.start_y
    length = math.hypot(ex, ey)

    def inside(t):
        t = np.atleast_1d(t)
        return _pad_sd(pad, seg.start_x + ex * t, seg.start_y + ey * t) <= -depth

    if length < 1e-9:
        return [(0.0, 1.0)] if inside(0.0)[0] else []
    # Only the stretch within the pad's reach of its centre can be on it.
    tc = ((pad.global_x - seg.start_x) * ex + (pad.global_y - seg.start_y) * ey) / length ** 2
    half = (_pad_reach(pad) + _CLIP_STEP) / length
    lo, hi = max(0.0, tc - half), min(1.0, tc + half)
    if lo >= hi:
        return []
    t = np.linspace(lo, hi, max(2, int(math.ceil((hi - lo) * length / _CLIP_STEP)) + 1))
    on = inside(t)
    spans, start = [], None

    def edge(a, b):                         # a and b straddle the boundary
        ia = bool(inside(a)[0])
        for _ in range(40):
            m = (a + b) / 2.0
            if bool(inside(m)[0]) == ia:
                a = m
            else:
                b = m
        return (a + b) / 2.0

    for k in range(len(t)):
        if on[k] and start is None:
            start = t[k] if k == 0 else edge(t[k - 1], t[k])
        if not on[k] and start is not None:
            spans.append((start, edge(t[k - 1], t[k])))
            start = None
    if start is not None:
        spans.append((start, float(t[-1])))
    return [(float(a), float(b)) for a, b in spans if b > a]


def _outside_join(seg, join) -> list:
    """The parts of `seg` (a member track) whose COPPER reaches off the join
    pad's real copper, as (x1, y1, x2, y2) centreline tuples. Member copper
    ON the pad is the pad. Clipped where the track's whole width -- not just
    its centreline -- is on the pad: a member leaving a tilted pad obliquely
    carries copper past the pad edge before its centreline crosses it, and a
    centreline clip left that copper out of the rule (a 30-degree join pad
    shipped new copper 0.108 mm from it at a 0.2 mm clearance)."""
    full = (seg.start_x, seg.start_y, seg.end_x, seg.end_y)
    if not _pad_on(join, seg.layer):
        return [full]
    spans = _on_pad_spans(seg, join, seg.width / 2.0)
    if not spans:
        return [full]                       # never crosses the pad
    ex, ey = seg.end_x - seg.start_x, seg.end_y - seg.start_y
    out, t = [], 0.0
    for t0, t1 in spans:
        if t0 > t + 1e-9:
            out.append((seg.start_x + ex * t, seg.start_y + ey * t,
                        seg.start_x + ex * t0, seg.start_y + ey * t0))
        t = max(t, t1)
    if t < 1.0 - 1e-9:
        out.append((seg.start_x + ex * t, seg.start_y + ey * t, seg.end_x, seg.end_y))
    return out


class _Piece:
    """A member track's off-pad part, shaped like a Segment for the stamp
    helpers."""
    __slots__ = ('start_x', 'start_y', 'end_x', 'end_y', 'width', 'layer', 'net_id')

    def __init__(self, xy, seg):
        self.start_x, self.start_y, self.end_x, self.end_y = xy
        self.width, self.layer, self.net_id = seg.width, seg.layer, seg.net_id


class _Collector:
    """Stands in for a GridObstacleMap while the obstacle_map stamp helpers
    draw the sense copper, so the join pad can be cut out of what they drew
    before it reaches the real map."""

    def __init__(self):
        self.cells, self.vias = [], []

    def add_blocked_cells_batch(self, cells):
        self.cells.append(np.asarray(cells, dtype=np.int32).reshape(-1, 3))

    add_static_blocked_cells_batch = add_blocked_cells_batch

    def add_blocked_vias_batch(self, vias):
        self.vias.append(np.asarray(vias, dtype=np.int32).reshape(-1, 2))

    def add_blocked_cell(self, gx, gy, layer):
        self.add_blocked_cells_batch([[gx, gy, layer]])

    def add_blocked_via(self, gx, gy):
        self.add_blocked_vias_batch([[gx, gy]])


def _pieces(b: SenseBranch, s) -> list:
    """`_outside_join(s, b.join)`, kept on the branch (member copper is held
    immutable for the run, and the clip of a round pad is bisected)."""
    cache = b.__dict__.setdefault('_pieces', {})
    if id(s) not in cache:
        cache[id(s)] = (s, _outside_join(s, b.join))
    return cache[id(s)][1]


def _branch_gap(b: SenseBranch, xs, ys, layer: str):
    """Edge gap (mm, negative inside) from each point to branch `b`'s copper
    on `layer` that is OFF its join pad: member track parts off the pad,
    member vias off it, sense pads."""
    from geometry_utils import via_on_layer
    best = np.full(xs.shape, np.inf)
    for s in b.segments:
        if s.layer != layer:
            continue
        for ax, ay, bx, by in _pieces(b, s):
            ex, ey = bx - ax, by - ay
            l2 = ex * ex + ey * ey
            t = (np.clip(((xs - ax) * ex + (ys - ay) * ey) / l2, 0.0, 1.0)
                 if l2 > 0 else np.zeros_like(xs))
            best = np.minimum(best, np.hypot(xs - (ax + t * ex), ys - (ay + t * ey)) - s.width / 2.0)
    for v in b.vias:
        if via_on_layer(v, layer) and not _via_on_join(v, b.join):
            best = np.minimum(best, np.hypot(xs - v.x, ys - v.y) - v.size / 2.0)
    for p in b.sense_pads:
        if _pad_on(p, layer):
            best = np.minimum(best, _pad_sd(p, xs, ys))
    return best


#: Unit offsets sampling a copper disc: its centre and three rings of 32.
_DISC_A = np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)
_DISC_R = np.repeat([1.0 / 3.0, 2.0 / 3.0, 1.0], 32)
_DISC_X = np.concatenate([[0.0], _DISC_R * np.tile(np.cos(_DISC_A), 3)])
_DISC_Y = np.concatenate([[0.0], _DISC_R * np.tile(np.sin(_DISC_A), 3)])


def _off_join_gap(b: SenseBranch, xs, ys, half: float, layers, allowance: float):
    """For discs of copper of radius `half` centred at (xs, ys): the edge gap
    from the part of each disc OFF the join pad -- reaching more than
    `allowance` past its copper -- to the branch's copper, min over `layers`
    (on a layer the join pad is not on, the whole disc counts); inf where no
    part of the disc is off the pad. Copper on the join pad is the join, so
    only what sticks out of it is held to the rule."""
    qx = (xs[:, None] + half * _DISC_X[None, :]).ravel()
    qy = (ys[:, None] + half * _DISC_Y[None, :]).ravel()
    shape = (len(xs), len(_DISC_X))
    off = (_pad_sd(b.join, qx, qy) > allowance).reshape(shape)
    best = np.full(len(xs), np.inf)
    for layer in layers:
        g = _branch_gap(b, qx, qy, layer).reshape(shape)
        if _pad_on(b.join, layer):
            g = np.where(off, g, np.inf)
        best = np.minimum(best, g.min(axis=1))
    return best


def _still_blocked(b: SenseBranch, xs, ys, half: float, net_half: float, clearance: float,
                   layers):
    """Mask of stamped cells the branch still blocks for a track or via of
    half-width `half` (the stamp's reserve) centred there: every cell whose
    copper misses the join pad, and every cell whose copper on the pad sticks
    out of it within `clearance` of the branch. A cell whose copper is all on
    the pad -- or overhangs it no more than copper of the net's own width
    `net_half` centred on a pad narrower than it must -- is the join."""
    allowance = _overhang_allowance(b.join, net_half)
    near = _pad_sd(b.join, xs, ys) <= half + allowance
    out = np.ones(len(xs), dtype=bool)
    if near.any():
        out[near] = _off_join_gap(b, xs[near], ys[near], half, layers, allowance) < clearance
    return out


def sense_obstacle_rows(b: SenseBranch, config, coord, layer_map, clearance: float,
                        extra_clearance: float = 0.0):
    """(cells (N, 3), vias (M, 2)): branch `b`'s member copper and sense pads
    stamped as foreign copper at `clearance` -- the same keep-outs
    build_base_obstacle_map draws for another net's copper -- minus every
    cell where new copper of `b`'s net would meet them only on the join pad."""
    from obstacle_map import (_add_segment_obstacle, _add_via_obstacle,
                              _add_pad_obstacle, _via_track_expansion_per_layer)
    from routing_utils import segment_blocked_cells_array
    import routing_defaults as defaults
    col = _Collector()
    join = b.join
    names = sorted(layer_map, key=layer_map.get)
    for s in b.segments:
        seg_w = s.width if s.width > 0 else config.get_track_width(s.layer)
        seg_clr = config.layer_clearance(s.layer, clearance)
        trk = config.track_obstacle_clearance(b.net_id, seg_clr)
        exp = config.route_reserve_width(s.layer) / 2 + seg_w / 2 + trk + extra_clearance
        vblk = config.via_size / 2 + seg_w / 2 + seg_clr + extra_clearance
        li = layer_map.get(s.layer)
        for xy in _outside_join(s, join):
            if li is None:              # a layer this run cannot route: vias only
                col.add_blocked_vias_batch(segment_blocked_cells_array(
                    *xy, vblk, coord.grid_step))
                continue
            _add_segment_obstacle(col, _Piece(xy, s), coord, li, exp, vblk)
    for v in b.vias:
        if _via_on_join(v, join):
            continue                    # a via on the join pad is the join
        vs = v.size if v.size > 0 else config.via_size
        full = _via_track_expansion_per_layer(vs, config, coord, clearance, extra_clearance)
        # Per layer of THIS map (a plane tap map has one layer).
        vte = [full[config.layers.index(n)] if n in config.layers else max(full)
               for n in names]
        vv = max(1.0, (vs / 2 + config.via_size / 2
                       + config.stack_clearance(clearance)) * coord.inv_step)
        # On the layers of its span only, as build_base_obstacle_map stamps
        # every other via (a blind via leaves the layers below it free).
        _add_via_obstacle(col, v, coord, len(names), vte, vv,
                          diagonal_margin=defaults.DIAGONAL_MARGIN, routing_layers=names)
    for p in b.sense_pads:
        _add_pad_obstacle(col, p, coord, layer_map, config, extra_clearance,
                          clearance_override=clearance)
    cells = (np.concatenate(col.cells) if col.cells else np.empty((0, 3), np.int32))
    vias = (np.concatenate(col.vias) if col.vias else np.empty((0, 2), np.int32))
    # Near the join pad the rule holds only the copper that sticks out of it:
    # a stamped cell whose new copper lies on the pad, or overhangs it clear
    # of the branch, is freed. The pad's own cell -- the router's terminal for
    # it (coord.to_grid of the pad) -- is always freed for tracks and vias, so
    # a join pad no wider than the track still has a cell to land on; copper
    # centred there is on the join whatever it must overhang.
    step = coord.grid_step
    centre = coord.to_grid(join.global_x, join.global_y)
    if len(cells):
        keep = np.ones(len(cells), dtype=bool)
        for lname, li in layer_map.items():
            if not _pad_on(join, lname):
                continue
            on = cells[:, 2] == li
            if on.any():
                clr = config.track_obstacle_clearance(
                    b.net_id, config.layer_clearance(lname, clearance)) + extra_clearance
                keep[on] = _still_blocked(
                    b, cells[on, 0] * step, cells[on, 1] * step,
                    config.route_reserve_width(lname) / 2.0,
                    config.get_net_track_width(b.net_id, lname) / 2.0, clr, [lname])
                keep[on] &= ~((cells[on, 0] == centre[0]) & (cells[on, 1] == centre[1]))
        cells = np.unique(cells[keep], axis=0)
    if len(vias):
        layers = sorted(set(names) | {s.layer for s in b.segments})
        keep = _still_blocked(b, vias[:, 0] * step, vias[:, 1] * step,
                              config.via_size / 2.0, config.via_size / 2.0,
                              config.stack_clearance(clearance) + extra_clearance, layers)
        keep &= ~((vias[:, 0] == centre[0]) & (vias[:, 1] == centre[1]))
        vias = np.unique(vias[keep], axis=0)
    return np.ascontiguousarray(cells, dtype=np.int32), np.ascontiguousarray(vias, dtype=np.int32)


def stamp_base_obstacles(obstacles, pcb_data, config, nets_to_route, coord, layer_map,
                         obstacle_clearance, extra_clearance: float = 0.0) -> None:
    """build_base_obstacle_map's half of the rule. The base map leaves out the
    copper of every net it is built to route (those nets' caches carry it and
    each is lifted for its own route), so a protected branch of such a net is
    stamped here as foreign copper, with the join pad cut out. Every map the
    router derives from a base -- the working map, a Phase-3 clone, a rescue
    window's pristine map, each per-net via rung (whose sub-builds run this
    body again) -- then holds it. A net NOT routed already sees the branch as
    that net's ordinary copper."""
    for b in protected_branches(pcb_data):
        if b.net_id not in nets_to_route:
            continue
        cells, vias = sense_obstacle_rows(b, config, coord, layer_map,
                                          obstacle_clearance(b.net_id), extra_clearance)
        if len(cells):
            obstacles.add_blocked_cells_batch(cells)
        if len(vias):
            obstacles.add_blocked_vias_batch(vias)


def stamp_plane_obstacles(obstacles, pcb_data, config, net_id: int, layers,
                          tracks: bool, vias: Optional[bool] = None,
                          extra_clearance: float = 0.0) -> None:
    """The plane path's half (plane_obstacle_builder): the via-placement map
    (`tracks` False) and the single-layer tap-routing map (`tracks` True,
    `layers` its one layer) built for `net_id` leave that net's copper out,
    so its protected branches are stamped in here, join pad cut out.

    `vias` defaults to `not tracks`: each of those two maps takes one half.
    plane_region_connector.build_base_obstacles routes AND places vias on one
    map over every routing layer, so it takes both, with its grid cushion as
    `extra_clearance` -- the cushion it adds to every foreign item."""
    from routing_config import GridCoord
    branches = protected_branches(pcb_data, net_id)
    if not branches:
        return
    stamp_vias = (not tracks) if vias is None else vias
    coord = GridCoord(config.grid_step)
    layer_map = {name: i for i, name in enumerate(layers)}
    clearance = (config.obstacle_clearance(net_id)
                 if hasattr(config, 'obstacle_clearance') else config.clearance)
    for b in branches:
        cells, vrows = sense_obstacle_rows(b, config, coord, layer_map, clearance,
                                           extra_clearance)
        if tracks and len(cells):
            obstacles.add_blocked_cells_batch(cells)
        if stamp_vias and len(vrows):
            obstacles.add_blocked_vias_batch(vrows)


def join_for_point(pcb_data, net_id: int, x: float, y: float, layer: Optional[str]):
    """The join pad to use instead of (x, y) on `layer`, when that point lies
    on a protected branch of `net_id` off its join pad (member copper or a
    sense pad), else None. KiCad's ratsnest knows nothing of the rule, so a
    link it reports can end on the sense trace (#572 forced oracle links,
    every kicad_oracle link). `layer` None (a via-point anchor) is any layer."""
    from geometry_utils import via_on_layer
    for b in protected_branches(pcb_data, net_id):
        if _in_pad(b.join, x, y, _EPS):
            return None
        on = (any((layer is None or s.layer == layer)
                  and _dist_seg(x, y, s.start_x, s.start_y, s.end_x, s.end_y)
                  <= s.width / 2.0 + _EPS for s in b.segments)
              or any((layer is None or via_on_layer(v, layer))
                     and math.hypot(x - v.x, y - v.y) <= v.size / 2.0 + _EPS
                     for v in b.vias)
              or any((layer is None or _pad_on(p, layer)) and _in_pad(p, x, y, _EPS)
                     for p in b.sense_pads))
        if on:
            return b.join
    return None


def join_link_end(pcb_data, net_id: int, end: tuple) -> tuple:
    """A link end `(x, y, layer, ...)` -- a #572 forced link's or a
    kicad_oracle link's -- moved to the join pad when it lies on one of the
    net's protected branches off that pad (`join_for_point`); otherwise
    `end` itself. The layer becomes one the join pad has; the rest of the
    tuple (the oracle's anchor kind) is kept."""
    join = join_for_point(pcb_data, net_id, end[0], end[1], end[2])
    if join is None:
        return end
    layer = end[2] if (end[2] in join.layers or '*.Cu' in join.layers) else next(
        (l for l in join.layers if l.endswith('.Cu')), end[2])
    return (join.global_x, join.global_y, layer) + tuple(end[3:])


def terminal_sense_dist(pcb_data, net_id: int, x1: float, y1: float, x2: float, y2: float,
                        layer: str, half_width: float) -> float:
    """The terminal-overlap gate's half of the rule: the smallest distance
    from the centreline of new copper (x1, y1)-(x2, y2) of `net_id` on
    `layer` (half-width `half_width`) to the EDGE of that net's protected
    sense copper, measured only for the new copper that is off the join pad
    (contact on the join pad is the join) -- the map's own measure
    (_still_blocked). 1e9 when the net has no protected branch."""
    branches = protected_branches(pcb_data, net_id)
    if not branches:
        return 1e9
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02) + 1)
    t = np.linspace(0.0, 1.0, n + 1)
    sx, sy = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
    best = 1e9
    for b in branches:
        near = np.zeros(len(sx), dtype=bool)
        if _pad_on(b.join, layer):
            allowance = _overhang_allowance(b.join, half_width)
            near = _pad_sd(b.join, sx, sy) <= half_width + allowance
            if near.any():
                # Copper reaching the pad: only what sticks out of it counts,
                # returned as the centreline distance the caller expects.
                g = _off_join_gap(b, sx[near], sy[near], half_width, [layer], allowance)
                best = min(best, float(g.min()) + half_width)
        if (~near).any():
            best = min(best, float(_branch_gap(b, sx[~near], sy[~near], layer).min()))
    return best
