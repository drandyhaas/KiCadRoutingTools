"""#595: where a signal track leaves its reference plane.

A track's return current runs in the plane copper next to it. Where the
adjacent plane layer has no fill under the track -- a split between two
pours, a void, a slot cut by a track or a row of clearance holes drawn on the
plane layer -- the return path detours, and where the fill under the track
changes from one net's pour to another's the reference changes. This module
MEASURES both on a routed board; it routes nothing and prices nothing.

For a track on copper layer L the reference layers are the copper layers
next to L in the stack that carry a pour (`kicad_parser.pour_zones`). The
fill under a point is read off the zone fill model (`plane_fill_model`, the
filled copper as KiCad's filler leaves it: zone clearance around foreign
tracks, vias and pads included), highest zone priority first. Each track is
sampled every `step` mm.
"""

import math
from typing import Dict, List, Optional

DEFAULT_STEP = 0.1


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


class FillReader:
    """Which net's pour copper lies under a point of a layer."""

    def __init__(self, pcb_data):
        from kicad_parser import pour_zones
        self.pcb_data = pcb_data
        self.by_layer: Dict[str, list] = {}
        for z in pour_zones(getattr(pcb_data, 'zones', None)):
            self.by_layer.setdefault(z.layer, []).append(z)
        for zs in self.by_layer.values():
            zs.sort(key=lambda z: -int(getattr(z, 'priority', 0) or 0))
        self._models: Dict[int, object] = {}
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

    def net_at(self, layer: str, x: float, y: float) -> Optional[str]:
        """The net whose fill covers (x, y) on `layer`, or None over a void."""
        for z in self.by_layer.get(layer, ()):
            m = self._model(z)
            if m is None:
                continue
            c = m.query_component(x, y)
            if c:
                return z.net_name
        return None


def _runs(values):
    """[(value, first index, last index)] for the runs of equal values."""
    out = []
    for i, v in enumerate(values):
        if out and out[-1][0] == v:
            out[-1][2] = i
        else:
            out.append([v, i, i])
    return out


def reference_plane_report(pcb_data, net_ids=None, step: float = DEFAULT_STEP) -> dict:
    """Measure every track of `net_ids` (default: every signal net, see
    `signal_net_ids`) against its reference planes.

    Per net: `length_mm` of track that has a reference layer, `void_mm` of
    it with no pour copper under it on ANY reference layer, `crossings` (the
    times a track enters such a void and comes out over copper again),
    `reference_changes` (the times the pour under a track changes net on its
    best reference layer, across a void or not), and `unreferenced_mm`, the
    track on layers with no poured neighbour at all. `rows` lists each
    crossing and change with its position.
    """
    readers = FillReader(pcb_data)
    refs = reference_layers(pcb_data)
    want = signal_net_ids(pcb_data) if net_ids is None else set(net_ids)
    by_net: Dict[str, dict] = {}
    rows: List[dict] = []
    for seg in pcb_data.segments:
        if seg.net_id not in want or getattr(seg, 'graphic', False):
            continue
        net = pcb_data.nets.get(seg.net_id)
        name = getattr(net, 'name', None) or f"net_{seg.net_id}"
        rec = by_net.setdefault(name, {'length_mm': 0.0, 'void_mm': 0.0, 'crossings': 0,
                                       'reference_changes': 0, 'unreferenced_mm': 0.0})
        length = math.hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y)
        layers = refs.get(seg.layer) or []
        if not layers:
            rec['unreferenced_mm'] += length
            continue
        rec['length_mm'] += length
        n = max(1, int(math.ceil(length / step)))
        pts = [(seg.start_x + (seg.end_x - seg.start_x) * k / n,
                seg.start_y + (seg.end_y - seg.start_y) * k / n) for k in range(n + 1)]
        under = {l: [readers.net_at(l, x, y) for x, y in pts] for l in layers}
        d = length / n
        # no copper under the track on any reference layer
        none = [all(under[l][k] is None for l in layers) for k in range(n + 1)]
        for v, a, b in _runs(none):
            if not v:
                continue
            # the void's edges lie between samples: half a step either side
            run = min(length, (b - a + 1) * d)
            rec['void_mm'] += run
            if a > 0 and b < n:         # entered and left within this track
                rec['crossings'] += 1
                rows.append({'net': name, 'layer': seg.layer, 'kind': 'void',
                             'at': [round(pts[a][0], 3), round(pts[a][1], 3)],
                             'to': [round(pts[b][0], 3), round(pts[b][1], 3)],
                             'length_mm': round(run, 4)})
        # the pour's net changing under the track, on its best reference
        best = max(layers, key=lambda l: sum(1 for v in under[l] if v is not None))
        last = None
        for k, v in enumerate(under[best]):
            if v is None:
                continue
            if last is not None and v != last[0]:
                rec['reference_changes'] += 1
                rows.append({'net': name, 'layer': seg.layer, 'kind': 'change',
                             'reference_layer': best, 'from': last[0], 'to_net': v,
                             'at': [round(pts[k][0], 3), round(pts[k][1], 3)]})
            last = (v, k)
    for rec in by_net.values():
        for k in ('length_mm', 'void_mm', 'unreferenced_mm'):
            rec[k] = round(rec[k], 4)
    tot = {k: round(sum(r[k] for r in by_net.values()), 4)
           for k in ('length_mm', 'void_mm', 'crossings', 'reference_changes',
                     'unreferenced_mm')}
    return {'step_mm': step, 'reference_layers': {l: v for l, v in refs.items() if v},
            'totals': tot, 'by_net': dict(sorted(by_net.items())), 'rows': rows,
            'unmodelled_zones': sorted(set(readers.unmodelled))}


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
