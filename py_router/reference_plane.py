"""#595: where a signal track leaves its reference plane.

A track's return current runs in the plane copper next to it. Where the
adjacent plane layer has no fill under the track -- a split between two
pours, a void, a slot cut by a track drawn on the plane layer -- the return
path detours, and where the fill under the track changes from one net's pour
to another's the reference changes. This module MEASURES both on a routed
board; it routes nothing and prices nothing.

For a track on copper layer L the reference layers are the copper layers
next to L in the stack that carry a pour (`kicad_parser.pour_zones`). The
fill under a point is read off the zone fill model (`plane_fill_model`, the
filled copper as KiCad's filler leaves it: zone clearance around foreign
tracks, vias and pads included), highest zone priority first. That model
does not keep one pour clear of ANOTHER (a sibling pour on the same layer
does not carve it, #483), so this reading adds what KiCad's filler does on
a split plane: a zone gives way, at the larger of the two zones' clearances,
to every zone of another net on its layer with a higher priority -- the
usual full-board ground pour under higher-priority power islands then reads
as a gap around each island, not as ground. The routing engine's fill model
is unchanged.

Tracks are followed as chains: a net's segments on a layer joined end to end
through points where exactly two of them meet, sampled every `step` mm. A
void is CROSSED when it starts and ends inside a chain (a track that ends in
a void enters it and does not come out). A void lying wholly inside the
clearance of ONE via, hole or pad on the reference layer is a CLEARANCE HOLE
and is counted apart from the voids (`hole_crossings`, `hole_mm`): the track
passes beside a drill, which is not a split in the plane.
"""

import math
from typing import Dict, List, Optional

DEFAULT_STEP = 0.1
_KEY = 3                 # chain joints: endpoints equal to 1 um


def reference_layers(pcb_data) -> Dict[str, List[str]]:
    """{copper layer: [the adjacent copper layers that carry a pour]}."""
    from kicad_parser import pour_zones
    copper = list(pcb_data.board_info.copper_layers)
    poured = {z.layer for z in pour_zones(getattr(pcb_data, 'zones', None))}
    out = {}
    for i, layer in enumerate(copper):
        out[layer] = [copper[j] for j in (i - 1, i + 1)
                      if 0 <= j < len(copper) and copper[j] in poured]
    return out


def _zone_clearance(z) -> float:
    c = getattr(z, 'clearance', None)
    return float(c) if c is not None and c > 0 else 0.0


class _Knockout:
    """A higher-priority zone of another net: its outline, less its
    cut-outs, grown by the zone-to-zone gap."""

    def __init__(self, zone, gap):
        import numpy as np
        self.zone, self.gap = zone, gap
        self.outline = [tuple(p) for p in zone.polygon]
        self.holes = [[tuple(p) for p in h] for h in (getattr(zone, 'holes', None) or [])
                      if len(h) >= 3]
        xs = [p[0] for p in self.outline]
        ys = [p[1] for p in self.outline]
        self.bbox = (min(xs) - gap, min(ys) - gap, max(xs) + gap, max(ys) + gap)
        edges = []
        for ring in [self.outline] + self.holes:
            for k in range(len(ring)):
                edges.append((ring[k - 1][0], ring[k - 1][1], ring[k][0], ring[k][1]))
        self.edges = np.asarray(edges, dtype=float)

    def covers(self, xs, ys):
        """Bool mask: the points this zone's knock-out covers."""
        import numpy as np
        from check_connected import points_in_polygon_mask
        out = np.zeros(len(xs), dtype=bool)
        b = self.bbox
        near = (xs >= b[0]) & (xs <= b[2]) & (ys >= b[1]) & (ys <= b[3])
        if not near.any() or len(self.outline) < 3:
            return out
        px, py = xs[near], ys[near]
        inside = points_in_polygon_mask(px, py, self.outline)
        for h in self.holes:
            inside &= ~points_in_polygon_mask(px, py, h)
        hit = inside.copy()
        if self.gap > 0:
            E = self.edges
            ax, ay, bx, by = E[:, 0], E[:, 1], E[:, 2], E[:, 3]
            dx, dy = bx - ax, by - ay
            ll = dx * dx + dy * dy
            ll[ll == 0] = 1e-18
            step = max(1, 2_000_000 // max(1, len(E)))
            for s in range(0, len(px), step):
                qx = px[s:s + step, None]
                qy = py[s:s + step, None]
                t = np.clip(((qx - ax) * dx + (qy - ay) * dy) / ll, 0.0, 1.0)
                d2 = (ax + t * dx - qx) ** 2 + (ay + t * dy - qy) ** 2
                hit[s:s + step] |= (d2.min(axis=1) < self.gap * self.gap)
        out[np.nonzero(near)[0]] = hit
        return out


class FillReader:
    """Which net's pour copper lies under points of a layer."""

    def __init__(self, pcb_data):
        from kicad_parser import pour_zones
        self.pcb_data = pcb_data
        self.by_layer: Dict[str, list] = {}
        for z in pour_zones(getattr(pcb_data, 'zones', None)):
            self.by_layer.setdefault(z.layer, []).append(z)
        for zs in self.by_layer.values():
            zs.sort(key=lambda z: -int(getattr(z, 'priority', 0) or 0))
        self._models: Dict[int, object] = {}
        self._knock: Dict[int, list] = {}
        self.unmodelled: List[str] = []

    def _model(self, zone):
        key = id(zone)
        if key not in self._models:
            from plane_fill_model import get_zone_model
            m = get_zone_model(self.pcb_data, zone)
            self._models[key] = m
            if m is None:
                self.unmodelled.append(f"{zone.net_name} on {zone.layer}")
        return self._models[key]

    def _knockouts(self, zone):
        """The zones of other nets on `zone`'s layer that outrank it."""
        key = id(zone)
        if key not in self._knock:
            p = int(getattr(zone, 'priority', 0) or 0)
            self._knock[key] = [
                _Knockout(h, max(_zone_clearance(zone), _zone_clearance(h)))
                for h in self.by_layer.get(zone.layer, ())
                if h is not zone and h.net_id != zone.net_id
                and int(getattr(h, 'priority', 0) or 0) > p]
        return self._knock[key]

    def nets_at(self, layer: str, xs, ys) -> List[Optional[str]]:
        """The net whose fill covers each point on `layer`, None over a void."""
        import numpy as np
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        out: List[Optional[str]] = [None] * len(xs)
        todo = np.ones(len(xs), dtype=bool)
        for z in self.by_layer.get(layer, ()):
            if not todo.any():
                break
            m = self._model(z)
            if m is None:
                continue
            i = ((xs - m.x0) / m.cell).astype(int)
            j = ((ys - m.y0) / m.cell).astype(int)
            inside = (i >= 0) & (i < m.nx) & (j >= 0) & (j < m.ny) & todo
            filled = np.zeros(len(xs), dtype=bool)
            filled[inside] = m.labels[i[inside], j[inside]] > 0
            for h in self._knockouts(z):
                if filled.any():
                    filled &= ~h.covers(xs, ys)
            for k in np.nonzero(filled)[0]:
                out[k] = z.net_name
            todo &= ~filled
        return out

    def net_at(self, layer: str, x: float, y: float) -> Optional[str]:
        return self.nets_at(layer, [x], [y])[0]


class _Holes:
    """Vias, drilled pads and pads with copper on a layer, to tell a void
    that is one item's clearance from a split."""

    def __init__(self, pcb_data, layer, clearance):
        from geometry_utils import via_on_layer
        self.clearance = clearance
        self.items = []         # (x, y, bounding reach, pad on the layer or None)
        for v in pcb_data.vias:
            if via_on_layer(v, layer):
                self.items.append((v.x, v.y, v.size / 2 + clearance, None))
        for fp in pcb_data.footprints.values():
            for p in fp.pads:
                cu = [l for l in (p.layers or []) if l.endswith('.Cu')]
                on = layer in cu or any('*' in l for l in cu)
                drill = getattr(p, 'drill', 0) or 0
                if on and getattr(p, 'pad_type', '') != 'np_thru_hole':
                    self.items.append((p.global_x, p.global_y,
                                       math.hypot(p.size_x, p.size_y) / 2 + clearance, p))
                elif drill > 0:
                    hx = p.hole_x if getattr(p, 'hole_x', None) is not None else p.global_x
                    hy = p.hole_y if getattr(p, 'hole_y', None) is not None else p.global_y
                    self.items.append((hx, hy, drill / 2 + clearance, None))
        self.cell = 2.0
        self.grid: Dict[tuple, list] = {}
        for it in self.items:
            r = it[2]
            for gx in range(int(math.floor((it[0] - r) / self.cell)),
                            int(math.floor((it[0] + r) / self.cell)) + 1):
                for gy in range(int(math.floor((it[1] - r) / self.cell)),
                                int(math.floor((it[1] + r) / self.cell)) + 1):
                    self.grid.setdefault((gx, gy), []).append(it)

    def one_covers(self, pts, slack) -> bool:
        """True when a single item's clearance holds every point."""
        from check_drc import point_to_pad_distance
        x0, y0 = pts[0]
        for x, y, reach, pad in self.grid.get((int(math.floor(x0 / self.cell)),
                                               int(math.floor(y0 / self.cell))), ()):
            r = reach + slack
            if not all((px - x) ** 2 + (py - y) ** 2 <= r * r for px, py in pts):
                continue
            if pad is None or all(point_to_pad_distance(px, py, pad)
                                  <= self.clearance + slack for px, py in pts):
                return True
        return False


def _runs(values):
    """[(value, first index, last index)] for the runs of equal values."""
    out = []
    for i, v in enumerate(values):
        if out and out[-1][0] == v:
            out[-1][2] = i
        else:
            out.append([v, i, i])
    return out


def _chains(segs):
    """Order one net's segments on one layer into chains through the points
    where exactly two meet. Returns [[(seg, reversed), ...], ...]."""
    def key(x, y):
        return (round(x, _KEY), round(y, _KEY))
    at: Dict[tuple, list] = {}
    for i, s in enumerate(segs):
        at.setdefault(key(s.start_x, s.start_y), []).append(i)
        at.setdefault(key(s.end_x, s.end_y), []).append(i)
    used = [False] * len(segs)

    def walk(i, from_key):
        chain = []
        cur = from_key
        while i is not None and not used[i]:
            used[i] = True
            s = segs[i]
            rev = key(s.start_x, s.start_y) != cur
            chain.append((s, rev))
            cur = key(s.start_x, s.start_y) if rev else key(s.end_x, s.end_y)
            nxt = [k for k in at[cur] if not used[k]]
            i = nxt[0] if len(at[cur]) == 2 and nxt else None
        return chain

    out = []
    for k, idx in at.items():                     # chains from their ends
        if len(idx) != 2:
            for i in idx:
                if not used[i]:
                    out.append(walk(i, k))
    for i, s in enumerate(segs):                  # what is left is closed
        if not used[i]:
            out.append(walk(i, key(s.start_x, s.start_y)))
    return out


def _sample(chain, step):
    """Points along a chain every `step` mm or less, and each point's
    distance along it."""
    pts, dist = [], []
    run = 0.0
    for s, rev in chain:
        x0, y0, x1, y1 = s.start_x, s.start_y, s.end_x, s.end_y
        if rev:
            x0, y0, x1, y1 = x1, y1, x0, y0
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(length / step)))
        for k in range(0 if not pts else 1, n + 1):
            pts.append((x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n))
            dist.append(run + length * k / n)
        run += length
    return pts, dist


def _run_length(dist, a, b):
    """A run's length: its samples' span plus half the gap to each
    neighbouring sample (the edges lie between samples)."""
    lo = (dist[a] - dist[a - 1]) / 2 if a > 0 else 0.0
    hi = (dist[b + 1] - dist[b]) / 2 if b + 1 < len(dist) else 0.0
    return dist[b] - dist[a] + lo + hi


def reference_plane_report(pcb_data, net_ids=None, step: float = DEFAULT_STEP,
                           reader=None) -> dict:
    """Measure every track of `net_ids` (default: every signal net, see
    `signal_net_ids`) against its reference planes. `reader` answers which
    net's fill lies under points (`nets_at(layer, xs, ys)`); default the
    zone fill model (`FillReader`).

    Per net: `length_mm` of track that has a reference layer; `void_mm` of it
    with no pour copper under it on ANY reference layer, and `crossings`, the
    times a track crosses such a void (enters it and comes out over copper
    again); `partial_crossings`, the times a track with two reference layers
    crosses a void on one of them while the other stays under it;
    `hole_crossings` and `hole_mm`, the same for voids that are one via's,
    hole's or pad's clearance (not in the counts above); `reference_changes`,
    the times the pour under a track changes net, on each reference layer;
    and `unreferenced_mm`, track on layers with no poured neighbour. `rows`
    lists each crossing and change with its position and reference layer."""
    reader = reader or FillReader(pcb_data)
    refs = reference_layers(pcb_data)
    want = signal_net_ids(pcb_data) if net_ids is None else set(net_ids)
    clr = {}
    for z in getattr(pcb_data, 'zones', None) or ():
        clr[z.layer] = max(clr.get(z.layer, 0.0), _zone_clearance(z))
    holes: Dict[str, _Holes] = {}

    def holes_on(layer):
        if layer not in holes:
            holes[layer] = _Holes(pcb_data, layer, clr.get(layer, 0.0))
        return holes[layer]

    by_net: Dict[str, dict] = {}
    rows: List[dict] = []
    groups: Dict[tuple, list] = {}
    for seg in pcb_data.segments:
        if seg.net_id not in want or getattr(seg, 'graphic', False):
            continue
        groups.setdefault((seg.net_id, seg.layer), []).append(seg)
    for (nid, layer), segs in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        net = pcb_data.nets.get(nid)
        name = getattr(net, 'name', None) or f"net_{nid}"
        rec = by_net.setdefault(name, {
            'length_mm': 0.0, 'void_mm': 0.0, 'crossings': 0, 'partial_crossings': 0,
            'hole_crossings': 0, 'hole_mm': 0.0, 'reference_changes': 0,
            'unreferenced_mm': 0.0})
        length = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)
        layers = refs.get(layer) or []
        if not layers:
            rec['unreferenced_mm'] += length
            continue
        rec['length_mm'] += length
        for chain in _chains(segs):
            pts, dist = _sample(chain, step)
            n = len(pts) - 1
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            under = {l: reader.nets_at(l, xs, ys) for l in layers}

            def is_hole(a, b, lays):
                return all(holes_on(l).one_covers(pts[a:b + 1], step) for l in lays)

            def row(kind, a, b, run, **extra):
                rows.append({'net': name, 'layer': layer, 'kind': kind,
                             'at': [round(pts[a][0], 3), round(pts[a][1], 3)],
                             'to': [round(pts[b][0], 3), round(pts[b][1], 3)],
                             'length_mm': round(run, 4), **extra})

            # no copper under the track on any reference layer
            none = [all(under[l][k] is None for l in layers) for k in range(n + 1)]
            for v, a, b in _runs(none):
                if not v:
                    continue
                run = _run_length(dist, a, b)
                crossed = a > 0 and b < n
                if is_hole(a, b, layers):
                    rec['hole_mm'] += run
                    if crossed:
                        rec['hole_crossings'] += 1
                        row('hole', a, b, run)
                    continue
                rec['void_mm'] += run
                if crossed:
                    rec['crossings'] += 1
                    row('void', a, b, run)
            # a void on one reference layer while another stays under it
            if len(layers) > 1:
                for l in layers:
                    part = [under[l][k] is None and not none[k] for k in range(n + 1)]
                    for v, a, b in _runs(part):
                        if not v or not (a > 0 and b < n):
                            continue
                        # entered from and left to copper on this layer
                        if under[l][a - 1] is None or under[l][b + 1] is None:
                            continue
                        run = _run_length(dist, a, b)
                        if is_hole(a, b, [l]):
                            rec['hole_crossings'] += 1
                            row('hole', a, b, run, reference_layer=l)
                            continue
                        rec['partial_crossings'] += 1
                        row('partial_void', a, b, run, reference_layer=l)
            # the pour's net changing under the track, on each reference layer
            for l in layers:
                last = None
                for k, v in enumerate(under[l]):
                    if v is None:
                        continue
                    if last is not None and v != last:
                        rec['reference_changes'] += 1
                        rows.append({'net': name, 'layer': layer, 'kind': 'change',
                                     'reference_layer': l, 'from': last, 'to_net': v,
                                     'at': [round(pts[k][0], 3), round(pts[k][1], 3)]})
                    last = v
    keys = ('length_mm', 'void_mm', 'crossings', 'partial_crossings', 'hole_crossings',
            'hole_mm', 'reference_changes', 'unreferenced_mm')
    for rec in by_net.values():
        for k in ('length_mm', 'void_mm', 'hole_mm', 'unreferenced_mm'):
            rec[k] = round(rec[k], 4)
    tot = {k: round(sum(r[k] for r in by_net.values()), 4) for k in keys}
    return {'step_mm': step, 'reference_layers': {l: v for l, v in refs.items() if v},
            'totals': tot, 'by_net': dict(sorted(by_net.items())), 'rows': rows,
            'unmodelled_zones': sorted(set(getattr(reader, 'unmodelled', ()) or ()))}


def signal_net_ids(pcb_data) -> set:
    """The nets measured by default: every net with a name that is neither a
    ground domain nor power (`list_nets.power_net_evidence`: a rail name,
    power pins on two or more parts, or a pour the board draws for it)."""
    from list_nets import power_net_evidence
    try:
        from net_queries import resolve_ground_domains
        ground = {nid for nid, refs in resolve_ground_domains(pcb_data).items() if refs}
    except Exception:                                           # noqa: BLE001
        ground = set()
    power = set(power_net_evidence(pcb_data))
    return {nid for nid, n in pcb_data.nets.items()
            if nid and getattr(n, 'name', None) and nid not in ground and nid not in power}
