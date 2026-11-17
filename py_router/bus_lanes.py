"""Laid lanes for a declared ordered bus (--bus-nets with --bus-lanes): the bus is laid, not searched for.

An ordered bus whose members join it along its run, from both sides (tributaries), cannot be held in order by the
corridor attraction (bus_corridor.py): a member pulled by a discount drifts toward its lane at a shallow angle, and
where it joins is wherever the search found cheapest. River routers assign the order first and the geometry after
(Leiserson & Pinter 1983; Dai, Dayan & Staepelaere 1991, SURF). So, for a bus given a centreline:

1. its centreline is the polyline the designer drew on a User layer (where the bus fits is the designer's knowledge);
2. its lanes are laid as copper on the bus layer before any net routes: one per member, offset from the centreline in
   the declared order one pitch apart (the spacing spreading to the fan pitch along the last segment, where the bus
   meets rows farther apart), each from where its member joins (its lane nearest its far end, else the nearest free
   spot downstream) to the centreline's end, a via at both;
3. the members' ramps (their far ends to their lanes' first vias, their lanes' last vias to their pads at the outlet)
   are routed as ordinary nets, the bus layer barred to them (a layer ban), so a ramp from the far side crosses the
   other lanes on another layer.

The order is then a construction: none out of order on the bus layer, the same copper every run, no other net in a
lane. Geometry here is in mm, board coordinates; the obstacle queries are the caller's (`legal`)."""
import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def _unit(dx: float, dy: float) -> Point:
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d else (1.0, 0.0)


def _normal(a: Point, b: Point) -> Point:
    """The left-hand normal of a -> b (board coordinates)."""
    tx, ty = _unit(b[0] - a[0], b[1] - a[1])
    return (-ty, tx)


def vertex_offsets(centre: Sequence[Point]) -> List[Point]:
    """The mitred offset direction at each vertex of the centreline: the unit offset there moves a lane one unit away
    from both adjacent segments (so the pitch holds through a bend); at the ends, the end segment's normal."""
    pts = list(centre)
    out = []
    for i in range(len(pts)):
        if i == 0:
            out.append(_normal(pts[0], pts[1]))
        elif i == len(pts) - 1:
            out.append(_normal(pts[-2], pts[-1]))
        else:
            n1, n2 = _normal(pts[i - 1], pts[i]), _normal(pts[i], pts[i + 1])
            mx, my = n1[0] + n2[0], n1[1] + n2[1]
            m = math.hypot(mx, my)
            if m < 1e-9:                                   # a reversal: no mitre
                out.append(n1)
                continue
            mx, my = mx / m, my / m
            cos_half = mx * n1[0] + my * n1[1]
            scale = 1.0 / max(cos_half, 0.25)              # a very sharp bend capped (4x)
            out.append((mx * scale, my * scale))
    return out


def lane_polylines(centre: Sequence[Point], n: int, pitch: float, fan_pitch: float = 0.0,
                   sign: int = 1) -> List[List[Point]]:
    """Member i's lane, i = 0 .. n-1 in the declared order: the centreline offset by (i - (n-1)/2) pitches along its
    mitred normals (sign -1 mirrors them), the last vertex's offset at fan_pitch instead when given."""
    pts = list(centre)
    if len(pts) < 2:
        raise ValueError('bus lanes: a centreline needs two points or more')
    offs = vertex_offsets(pts)
    c = (n - 1) / 2.0
    lanes = []
    for i in range(n):
        lane = []
        for k, ((x, y), (ox, oy)) in enumerate(zip(pts, offs)):
            p = fan_pitch if (fan_pitch > 0 and k == len(pts) - 1) else pitch
            d = sign * (i - c) * p
            lane.append((x + ox * d, y + oy * d))
        lanes.append(lane)
    return lanes


def outlet_sign(centre: Sequence[Point], outlets: Sequence[Optional[Point]]) -> int:
    """+1 if the members, in the declared order, lie along the centreline's left-hand normal at its end (the outlet,
    where every member leaves the bus: its copper there, `outlets[i]`, None if unknown), -1 if against it; a tie +1.
    Only the outlet votes: the members' far ends may lie on both sides of the run (tributaries)."""
    pts = list(centre)
    ex, ey = pts[-1]
    nx, ny = _normal(pts[-2], pts[-1])
    n = len(outlets)
    c = (n - 1) / 2.0
    vote = sum((i - c) * ((o[0] - ex) * nx + (o[1] - ey) * ny) for i, o in enumerate(outlets) if o is not None)
    return -1 if vote < 0 else 1


def _project(poly: Sequence[Point], p: Point) -> Tuple[int, float, float]:
    """(segment index, its parameter 0..1, distance from p) of the point of `poly` nearest to p."""
    best = (0, 0.0, float('inf'))
    for k, (a, b) in enumerate(zip(poly, poly[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
        d = math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])
        if d < best[2]:
            best = (k, t, d)
    return best


def _at(poly: Sequence[Point], k: int, t: float) -> Point:
    a, b = poly[k], poly[k + 1]
    return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))


def from_point(poly: Sequence[Point], p: Point) -> List[Point]:
    """The polyline from its point nearest p (taken as p) to its end."""
    k, t, _ = _project(poly, p)
    q = _at(poly, k, t)
    rest = [q] + list(poly[k + 1:])
    if len(rest) >= 2 and math.hypot(rest[1][0] - q[0], rest[1][1] - q[1]) < 1e-9:
        rest = rest[1:]
    return rest


def walk(poly: Sequence[Point], k: int, t: float, step: float) -> Optional[Tuple[int, float]]:
    """The place `step` mm downstream (toward the polyline's end) of (k, t), or None past its end."""
    while True:
        a, b = poly[k], poly[k + 1]
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        left = (1.0 - t) * L
        if step <= left + 1e-12:
            return (k, t + (step / L if L else 1.0))
        step -= left
        k, t = k + 1, 0.0
        if k >= len(poly) - 1:
            return None


def drop_point(lane: Sequence[Point], far: Point, legal: Callable[[Point], bool], step: float = 0.1,
               reach: float = 10.0) -> Optional[Tuple[Point, List[Point]]]:
    """Where a member joins its lane: the lane's point nearest its far end, else the first point downstream within
    `reach` mm (every `step` mm) that `legal` accepts; (the drop point, the lane from there to its end), or None.
    Only downstream: a lane never reaches back past its member's far end, so no ramp runs against the flow."""
    k, t, _ = _project(lane, far)
    travelled = 0.0
    while travelled <= reach + 1e-9:
        p = _at(lane, k, t)
        if legal(p):
            rest = [p] + [q for q in lane[k + 1:]]
            if len(rest) >= 2 and math.hypot(rest[1][0] - p[0], rest[1][1] - p[1]) < 1e-9:
                rest = rest[1:]
            return p, rest
        nxt = walk(lane, k, t, step)
        if nxt is None:
            return None
        k, t = nxt
        travelled += step
    return None


def auto_pitch(track_width: float, clearance: float, via_size: float) -> float:
    """The smallest lane pitch that leaves room for a member's via on its lane beside the next lane:
    via radius + the next track's half width + clearance, and never under one track + clearance."""
    return round(max(track_width + clearance, via_size / 2 + track_width / 2 + clearance), 4)


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])


def clear_of_lanes(p: Point, radius: float, lanes: Dict[int, Sequence[Point]], own: int, half_width: float,
                   clearance: float, vias: Sequence[Tuple[Point, int]], via_size: float) -> bool:
    """A via of member `own` (radius) at p clears every other member's laid lane and via by `clearance`."""
    for m, poly in lanes.items():
        if m == own:
            continue
        for a, b in zip(poly, poly[1:]):
            if point_segment_distance(p, a, b) < radius + half_width + clearance - 1e-9:
                return False
    for q, m in vias:
        if m != own and math.hypot(q[0] - p[0], q[1] - p[1]) < radius + via_size / 2 + clearance - 1e-9:
            return False
    return True


# --------------------------------------------------------------------------
# On a board: which buses are laid, their layer bans, the laying, the trim.

def laid_specs(config) -> List[Tuple[List[str], str, str]]:
    """(the bus's net names in order, its guide layer, its bus layer) for each declared bus with --bus-lanes."""
    out = []
    lanes = list(getattr(config, 'bus_lanes', None) or [])
    for i, nets in enumerate(getattr(config, 'bus_nets', None) or []):
        spec = lanes[i] if i < len(lanes) else None
        if spec:
            out.append((list(nets), spec[0], spec[1]))
    return out


def layer_bans(config) -> List[dict]:
    """Each laid bus's members barred from its bus layer while they route (their ramps cross the other lanes on
    another layer), in net_keepouts' layer-ban shape; `bus_lanes` marks them so the audit leaves the lanes themselves
    alone (they ARE on that layer, by construction)."""
    return [{'name': f'bus lanes {j}', 'rule': 'bus-lanes', 'polygon': None, 'holes': [], 'layers': {layer},
             'tracks_allowed': False, 'vias_allowed': True, 'nets': frozenset(nets), 'layer_ban': True,
             'bus_lanes': True}
            for j, (nets, _guide, layer) in enumerate(laid_specs(config))]


def _centreline(content: str, layer: str) -> List[Point]:
    from kicad_parser import parse_guide_paths
    paths = [g for g in parse_guide_paths(content, layer) if not g.is_closed and len(g.points) >= 2]
    if len(paths) != 1:
        raise ValueError(f'bus lanes: {layer} must hold one open polyline (the bus\'s centreline), found {len(paths)}')
    return [tuple(p) for p in paths[0].points]


def _copper_points(pcb_data, nid: int) -> List[Point]:
    pts = [(p.global_x, p.global_y) for p in pcb_data.pads_by_net.get(nid, [])]
    for s in pcb_data.segments:
        if s.net_id == nid:
            pts += [(s.start_x, s.start_y), (s.end_x, s.end_y)]
    return pts


def outlet_point(pcb_data, nid: int, end: Point) -> Optional[Point]:
    """Where a member leaves the bus, for the side vote: its pad nearest the centreline's end, else (a net with no
    pad) its copper point nearest it. A short member's nearest copper point can be its far end; its pad there is
    its outlet."""
    pads = [(p.global_x, p.global_y) for p in pcb_data.pads_by_net.get(nid, [])]
    pts = pads or _copper_points(pcb_data, nid)
    return min(pts, key=lambda p: math.hypot(p[0] - end[0], p[1] - end[1])) if pts else None


def lay(pcb_data, config, content: str = None, verbose: bool = True, board_file: str = None) -> Dict[str, object]:
    """Lay every bus with --bus-lanes on `pcb_data` (its segments and vias, locked), before any obstacle map is built.
    Returns {'segments': [...], 'vias': [...], 'laid': {net_id: {...}}, 'skipped': [...]}: the new copper, to be
    written with the run's (the swap channels), and per member its drop, its lane's end and its lane."""
    from kicad_parser import Segment, Via
    from net_keepouts import stamp
    from obstacle_map import build_base_obstacle_map
    from routing_config import GridCoord
    out = {'segments': [], 'vias': [], 'laid': {}, 'skipped': []}
    specs = laid_specs(config)
    if not specs:
        return out
    if content is None:                               # read only when a bus is laid: callers may hand an in-memory board
        with open(board_file or pcb_data.source_path) as f:
            content = f.read()
    by_name = {n.name: nid for nid, n in pcb_data.nets.items()}
    coord = GridCoord(config.grid_step)
    for nets, guide, layer in specs:
        ids = [by_name.get(n) for n in nets]
        if None in ids:
            raise ValueError(f'bus lanes: unknown net(s) {[n for n, i in zip(nets, ids) if i is None]}')
        if layer not in config.layers:
            raise ValueError(f'bus lanes: the bus layer {layer} is not a routing layer ({", ".join(config.layers)})')
        centre = _centreline(content, guide)
        end = centre[-1]
        widths = {i: config.get_net_track_width(i, layer) for i in ids}
        vias = {i: config.net_via(i) for i in ids}
        clear = {i: config.get_net_clearance(i) for i in ids}
        pitch = config.bus_lane_pitch or max(auto_pitch(widths[i], clear[i], vias[i][0]) for i in ids)
        outlets = [outlet_point(pcb_data, i, end) for i in ids]
        sign = outlet_sign(centre, outlets)
        polys = lane_polylines(centre, len(ids), pitch, config.bus_fan_pitch, sign)
        reach = getattr(config, 'bus_drop_reach', 0.0) or len(ids) * pitch     # the bus's width: what blocks a drop
        li = config.layers.index(layer)
        placed_lanes, placed_vias = {}, []

        def via_ok(p, i):
            gx, gy = coord.to_grid(p[0], p[1])
            if obstacles.is_via_blocked(gx, gy):
                return False
            return clear_of_lanes(p, vias[i][0] / 2, placed_lanes, i, widths[i] / 2, clear[i], placed_vias, vias[i][0])

        def lane_ok(poly, i):
            for a, b in zip(poly, poly[1:]):
                n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / config.grid_step))
                for k in range(n + 1):
                    x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
                    gx, gy = coord.to_grid(x, y)
                    if obstacles.is_blocked(gx, gy, li):
                        return (x, y)
            return None

        for k, (i, poly) in enumerate(zip(ids, polys)):
            # every other net's copper an obstacle, the other members' pre-existing copper (their pads, their stubs,
            # their rows at the outlet) included: a map per member, without its own copper. The lanes laid in this
            # pass are left out of it and checked exactly instead (clear_of_lanes: via radius, track width,
            # clearance), as the map's via blocking round a track is coarser than the lanes' pitch allows for.
            _laid_ids = {id(x) for x in out['segments']} | {id(x) for x in out['vias']}
            _all_s, _all_v = pcb_data.segments, pcb_data.vias
            pcb_data.segments = [x for x in _all_s if id(x) not in _laid_ids]
            pcb_data.vias = [x for x in _all_v if id(x) not in _laid_ids]
            try:
                obstacles = build_base_obstacle_map(pcb_data, config, [i], net_clearances=config.net_clearances)
            finally:
                pcb_data.segments, pcb_data.vias = _all_s, _all_v
            if any(s.net_id == i and s.layer == layer and getattr(s, 'locked', False) and
                   min(point_segment_distance((s.start_x, s.start_y), a, b) for a, b in zip(poly, poly[1:])) < 0.05 and
                   min(point_segment_distance((s.end_x, s.end_y), a, b) for a, b in zip(poly, poly[1:])) < 0.05
                   for s in pcb_data.segments):
                out['skipped'].append(([pcb_data.nets[i].name], 'already laid (its lane is on the board: a nested run)'))
                continue
            pts = _copper_points(pcb_data, i)
            if not pts:
                out['skipped'].append(([pcb_data.nets[i].name], 'no copper'))
                continue
            far = max(pts, key=lambda p: math.hypot(p[0] - end[0], p[1] - end[1]))
            # the member's own net-filtered keep-outs (a port's area, say) stamped while its copper is placed
            with stamp(obstacles, pcb_data, (i,), config):
                # the lane's end, nearest legal point upstream of the centreline's end
                back = drop_point(list(reversed(poly)), poly[-1], lambda p: via_ok(p, i),
                                  step=config.grid_step, reach=reach / 2)
                if back is None:
                    raise ValueError(f'bus lanes: {pcb_data.nets[i].name}: no room for its lane\'s end via near '
                                     f'({poly[-1][0]:.2f}, {poly[-1][1]:.2f})')
                tail = back[0]
                upstream = list(reversed(back[1]))           # the lane from its start to the end via
                # where it joins: its via fits, and its lane from there to the end via is clear
                found = drop_point(upstream, far, lambda p: via_ok(p, i) and
                                   math.hypot(p[0] - tail[0], p[1] - tail[1]) > 2 * vias[i][0] and
                                   lane_ok(from_point(upstream, p), i) is None,
                                   step=config.grid_step, reach=reach)
                if found is None:
                    blocked = lane_ok(from_point(upstream, far), i)
                    out['skipped'].append(([pcb_data.nets[i].name],
                                           'it joins at the outlet: no lane' if blocked is None else
                                           f'no clear lane within {reach:.1f} mm downstream of where it joins (blocked at '
                                           f'({blocked[0]:.2f}, {blocked[1]:.2f}))'))
                    continue
                drop, lane = found
            placed_lanes[i] = lane
            placed_vias += [(drop, i), (tail, i)]
            segs = [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=widths[i], layer=layer,
                            net_id=i, locked=True) for a, b in zip(lane, lane[1:])]
            # through vias (the default: a lane's ramps reach it from the outer layers; not a limit of the design)
            vs = [Via(x=p[0], y=p[1], size=vias[i][0], drill=vias[i][1], layers=['F.Cu', 'B.Cu'],
                      net_id=i, locked=True) for p in (drop, tail)]
            # the lane part of its member's terminals: the router connects a net's terminals (its tree from their
            # straight distances), and copper without one is only a place it may launch from, so a member could
            # bypass its lane (its far end straight to its outlet pads). Both lane vias are terminals (connectivity:
            # a group's flagged vias are all endpoints), so the member's tree is far end -> drop via and end via ->
            # outlet pads, the lane joining the two. On the vias themselves: nothing synthetic in pads_by_net.
            for v in vs:
                v.bus_lane_terminal = True
            pcb_data.segments.extend(segs)
            pcb_data.vias.extend(vs)
            out['segments'] += segs
            out['vias'] += vs
            out['laid'][i] = {'drop': drop, 'end': tail, 'lane': lane, 'layer': layer}
        if verbose:
            names = [pcb_data.nets[i].name for i in out['laid'] if i in ids]
            out['skipped'] = [x for x in out['skipped']]
            print(f"  bus lanes on {layer} along {guide}: {len(names)} laid, pitch {pitch:.3f}mm"
                  + (f", fan {config.bus_fan_pitch:.3f}mm" if config.bus_fan_pitch else '')
                  + f": {' '.join(names)}")
            for nets_s, why in out['skipped']:
                print(f"    {' '.join(nets_s)}: not laid ({why})")
    pcb_data._bus_laid = out
    return out


def trim_unused_drops(laid: Dict[str, object], copper_segments: Sequence, copper_vias: Sequence,
                      pads_by_net: Dict[int, list]) -> List[object]:
    """A laid drop via its ramp did not use (the ramp joined the lane by a via of its own, downstream): the via is
    removed and the lane starts at the ramp's via instead. `copper_*` are every segment and via of the board as it will
    ship (the laid ones among them); returns the laid vias removed (the caller drops them from what it writes)."""
    removed = []
    for nid, info in laid.get('laid', {}).items():
        drop, layer = info['drop'], info['layer']
        drop_via = next((v for v in copper_vias if v.net_id == nid and math.hypot(v.x - drop[0], v.y - drop[1]) < 1e-6),
                        None)
        if drop_via is None:
            continue
        r = drop_via.size / 2

        def touches(s):
            return min(math.hypot(s.start_x - drop[0], s.start_y - drop[1]),
                       math.hypot(s.end_x - drop[0], s.end_y - drop[1])) <= r
        used = any(s.net_id == nid and s.layer != layer and touches(s) for s in copper_segments) or any(
            math.hypot(p.global_x - drop[0], p.global_y - drop[1]) <= r for p in pads_by_net.get(nid, []))
        if used:
            continue
        lane_seg = next((s for s in copper_segments if s.net_id == nid and s.layer == layer and
                         math.hypot(s.start_x - drop[0], s.start_y - drop[1]) < 1e-6), None)
        joins = [v for v in copper_vias if v.net_id == nid and v is not drop_via and lane_seg is not None and
                 point_segment_distance((v.x, v.y), (lane_seg.start_x, lane_seg.start_y),
                                        (lane_seg.end_x, lane_seg.end_y)) < 1e-3]
        if lane_seg is None or not joins:
            continue                                  # nothing joins on its first segment: left as laid
        j = min(joins, key=lambda v: math.hypot(v.x - drop[0], v.y - drop[1]))
        lane_seg.start_x, lane_seg.start_y = j.x, j.y
        info['drop'] = (j.x, j.y)
        removed.append(drop_via)
    return removed
