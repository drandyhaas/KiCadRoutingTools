"""How many plane-net pads each tap via serves, on a board as it ships (#1896).

A pad of a plane net reaches its pour through a via: one of its own (a
fan-out dogbone, a via in the pad, a repair tap), or a neighbour's, joined to
it by a surface track. The second is cheaper, and for a supply ball it is a
weaker rail: the shared via carries several pads' current, and the surface
track adds inductance. This module counts it on the copper itself, whoever
laid it -- the fan-out, the plane repair's taps and straps, its join gate,
the oracle's reconnect -- so a shared via is visible without reading a
per-pad log.

A pad REACHES a via when the via sits in the same piece of copper as the pad
on the pad's own layer: tracks, pads and vias of the net on that layer,
joined where their copper overlaps. A via joins the pieces it touches on that
layer only -- through a via every pad of the net meets every other in the
plane, which is the question this does not ask. A via counts as a TAP via when
it has copper on a layer the net is poured on (other than the pad's own); a
via that only changes a signal's layer is not a tap.

Through-hole pads are left out: their barrel is their own via. Zone fill on
the pad's layer is not surface copper here, so a pad joined to a via only by a
pour on its own layer reaches none.

Report-only: it changes no copper. `tap_via_census` is the measurement,
`tap_via_line` its console line. route.py's step reads it on the board it
ships, for the nets its plane finalize served (both fronts: the engine
measures it, under `return_results` on the board the GUI will apply), and
repair_planes.py on the board it writes, after the oracle's reconnect.
"""
from __future__ import annotations

import math
from collections import defaultdict
from typing import Dict, Iterable, List, Optional

# Copper that overlaps by less than KiCad's 1 nm grid is not joined.
_EPS = 1e-6
_CELL = 2.0


class _DSU:
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, a):
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        a, b = self.find(a), self.find(b)
        if a != b:
            self.p[b] = a


def _pad_copper_layers(pad) -> List[str]:
    return [l for l in (getattr(pad, 'layers', None) or [])
            if l.endswith('.Cu') and '*' not in l]


def _cells(x0, y0, x1, y1):
    for gx in range(int(math.floor(x0 / _CELL)), int(math.floor(x1 / _CELL)) + 1):
        for gy in range(int(math.floor(y0 / _CELL)), int(math.floor(y1 / _CELL)) + 1):
            yield gx, gy


def _layer_pieces(pads, segs, vias):
    """Union-find over one net's copper on one layer. Nodes: pads (0..),
    then segments, then vias. Returns (dsu, offsets)."""
    from geometry_utils import point_to_segment_distance, segment_to_segment_distance
    from check_connected import track_reaches_pad, _point_in_pad

    n_p, n_s = len(pads), len(segs)
    dsu = _DSU(n_p + n_s + len(vias))
    grid = defaultdict(list)          # cell -> [(kind, index)]
    for i, p in enumerate(pads):
        r = math.hypot(p.size_x, p.size_y) / 2
        for c in _cells(p.global_x - r, p.global_y - r, p.global_x + r, p.global_y + r):
            grid[c].append(('p', i))
    for i, s in enumerate(segs):
        h = s.width / 2
        for c in _cells(min(s.start_x, s.end_x) - h, min(s.start_y, s.end_y) - h,
                        max(s.start_x, s.end_x) + h, max(s.start_y, s.end_y) + h):
            grid[c].append(('s', i))
    for i, v in enumerate(vias):
        r = v.size / 2
        for c in _cells(v.x - r, v.y - r, v.x + r, v.y + r):
            grid[c].append(('v', i))

    seen = set()
    for items in grid.values():
        for a in range(len(items)):
            for b in range(a + 1, len(items)):
                ka, kb = items[a], items[b]
                if ka[0] > kb[0]:
                    ka, kb = kb, ka          # order: 'p' < 's' < 'v'
                key = (ka, kb)
                if key in seen:
                    continue
                seen.add(key)
                (ta, ia), (tb, ib) = ka, kb
                touch = False
                if ta == 'p' and tb == 's':
                    touch = track_reaches_pad(segs[ib], pads[ia], tolerance=_EPS)
                elif ta == 'p' and tb == 'v':
                    v = vias[ib]
                    touch = _point_in_pad(v.x, v.y, pads[ia], margin=v.size / 2 - _EPS)
                elif ta == 's' and tb == 's':
                    s1, s2 = segs[ia], segs[ib]
                    touch = (segment_to_segment_distance(
                        s1.start_x, s1.start_y, s1.end_x, s1.end_y,
                        s2.start_x, s2.start_y, s2.end_x, s2.end_y)
                        <= (s1.width + s2.width) / 2 - _EPS)
                elif ta == 's' and tb == 'v':
                    s, v = segs[ia], vias[ib]
                    touch = (point_to_segment_distance(
                        v.x, v.y, s.start_x, s.start_y, s.end_x, s.end_y)
                        <= (s.width + v.size) / 2 - _EPS)
                if touch:
                    off = {'p': 0, 's': n_p, 'v': n_p + n_s}
                    dsu.union(off[ta] + ia, off[tb] + ib)
    return dsu, n_p, n_s


def tap_via_census(pcb, net_names: Iterable[str],
                   segments: Optional[list] = None,
                   vias: Optional[list] = None) -> Dict:
    """For every SMD pad of the nets `net_names`, the tap vias it reaches
    through surface copper (module docstring), on the copper `segments` /
    `vias` (default: the board's own). `pcb` supplies the pads, the nets and
    the pours.

    Returns {'pads': pads that reach a tap via, 'vias': tap vias that serve a
    pad, 'shared_vias': those serving more than one, 'max_pads_per_via',
    'pads_on_shared_vias': pads reaching a shared via (each once), 'shared':
    [{'net', 'at': [x, y], 'pads': [REF.PAD, ...]}, ...] most pads first}."""
    from geometry_utils import via_on_layer
    from kicad_parser import pad_is_plated_through

    segments = pcb.segments if segments is None else segments
    vias = pcb.vias if vias is None else vias
    by_name = {n.name: nid for nid, n in pcb.nets.items()}
    net_ids = {by_name[n] for n in net_names if n in by_name}
    zone_layers = defaultdict(set)
    for z in getattr(pcb, 'zones', None) or []:
        if z.net_id in net_ids and str(getattr(z, 'layer', '')).endswith('.Cu'):
            zone_layers[z.net_id].add(z.layer)
    segs_by = defaultdict(list)
    for s in segments:
        if s.net_id in net_ids and not getattr(s, 'graphic', False):
            segs_by[(s.net_id, s.layer)].append(s)
    vias_by = defaultdict(list)
    for v in vias:
        if v.net_id in net_ids:
            vias_by[v.net_id].append(v)

    served: Dict[tuple, List[str]] = {}       # (net, x, y) -> [pads]
    pads_reaching = set()
    for nid in sorted(net_ids):
        planes = zone_layers.get(nid)
        if not planes:
            continue
        by_layer = defaultdict(list)
        for p in (pcb.pads_by_net.get(nid) or []):
            if pad_is_plated_through(p) or getattr(p, 'pad_type', '') == 'np_thru_hole':
                continue
            for l in _pad_copper_layers(p):
                by_layer[l].append(p)
        for layer, pads in by_layer.items():
            lvias = [v for v in vias_by[nid] if via_on_layer(v, layer)]
            if not lvias:
                continue
            segs = segs_by.get((nid, layer), [])
            dsu, n_p, n_s = _layer_pieces(pads, segs, lvias)
            taps_in = defaultdict(list)
            for i, v in enumerate(lvias):
                if any(via_on_layer(v, zl) for zl in planes if zl != layer):
                    taps_in[dsu.find(n_p + n_s + i)].append(v)
            for i, p in enumerate(pads):
                name = f"{p.component_ref}.{p.pad_number}"
                for v in taps_in.get(dsu.find(i), ()):
                    key = (nid, round(v.x, 4), round(v.y, 4))
                    lst = served.setdefault(key, [])
                    if name not in lst:
                        lst.append(name)
                    pads_reaching.add((nid, name))
    shared = [(k, v) for k, v in served.items() if len(v) > 1]
    shared.sort(key=lambda kv: (-len(kv[1]), kv[0]))
    return {
        'pads': len(pads_reaching),
        'vias': len(served),
        'shared_vias': len(shared),
        'max_pads_per_via': max((len(v) for v in served.values()), default=0),
        'pads_on_shared_vias': len({(k[0], p) for k, v in shared for p in v}),
        'shared': [{'net': pcb.nets[k[0]].name, 'at': [k[1], k[2]],
                    'pads': sorted(v)} for k, v in shared],
    }


def tap_via_line(rec: Optional[Dict], limit: int = 6) -> Optional[str]:
    """The `Tap vias:` console line for a census, None when no pad reaches a
    tap via."""
    if not rec or not rec.get('vias'):
        return None
    line = (f"Tap vias: {rec['vias']} tap via(s) serve {rec['pads']} plane-net "
            f"pad(s); {rec['shared_vias']} serve more than one pad")
    if rec['shared_vias']:
        line += (f" ({rec['pads_on_shared_vias']} pad(s), up to "
                 f"{rec['max_pads_per_via']} on one via): "
                 + "; ".join(f"{'+'.join(r['pads'])} at ({r['at'][0]:.2f}, "
                             f"{r['at'][1]:.2f})" for r in rec['shared'][:limit])
                 + (" ..." if len(rec['shared']) > limit else ""))
    return line
