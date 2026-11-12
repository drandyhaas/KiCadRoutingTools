"""#1847: voltage drop at each load, and current in each track and via, of a
routed power net -- with every load drawing at once.

`plane_resistance.analyze_power_trace_net` names a net's narrowest segment
and its IPC rating. It does not know the loads, so it cannot say how many
millivolts a load loses, or that one via carries the current of three loads.
This module builds the net's copper into a resistor network and solves it:

  * a track segment is a resistor, rho * length / (width * thickness), its
    thickness the layer's own copper weight (`stackup_copper_oz`);
  * a via is a barrel resistor between each pair of neighbouring layers it is
    joined on, rho * height / (pi * (drill + plating) * plating);
  * a pad is a junction: every track end and via on its copper meets there.

The sources are held at 0 V of drop and each load draws its stated current;
the node voltages are the drops. Report-only: nothing reads the result to
route or to gate.

A copper pour is a sheet: each pour of the net is read from the fill model
(`plane_fill_model.get_fill_models`, the copper the pour really lays around
foreign items), coarsened to a square mesh, and solved as resistors of one
sheet resistance, rho / thickness, between neighbouring cells. A via, a pad
and a track end standing on the pour meet the cells under their copper.

What is NOT in the solve, and is said so in the result: a pour whose fill
model could not be built, thermal-relief spokes (a pad's contact with a pour
is taken as solid, so its drop reads low by the spokes' resistance), and
footprint graphic copper.
"""
import math
from typing import Dict, Iterable, List, Optional, Tuple

from plane_resistance import (COPPER_RESISTIVITY, OZ_TO_METERS, stackup_copper_oz,
                              calculate_max_current_ipc2152,
                              calculate_max_current_ipc2221)

#: plated barrel wall of a via, mm (IPC-6012 class 2 asks an average of 20 um;
#: fabs quote 18-25). A stated assumption: `via_plating_mm` overrides it.
DEFAULT_VIA_PLATING_MM = 0.025
#: board thickness assumed for a via's height when the board has no stackup
DEFAULT_BOARD_THICKNESS_MM = 1.6
#: side of a pour's mesh cell, mm: fine enough that a 0.5 mm neck is two cells
DEFAULT_POUR_MESH_MM = 0.25
#: two track ends, or a track end and a via or pad centre, closer than this
#: are one junction (the connectivity checker's coincidence scale)
JOIN_TOL_MM = 0.002


def pad_ref(pad) -> str:
    return f"{pad.component_ref}.{pad.pad_number}"


def _layer_heights(pcb_data) -> Dict[str, float]:
    """{copper layer: depth of its centre below the top copper, mm}."""
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    stackup = getattr(pcb_data.board_info, 'stackup', None) or []
    depth, out, started = 0.0, {}, False
    for entry in stackup:
        t = float(getattr(entry, 'thickness', 0.0) or 0.0)
        if entry.layer_type == 'copper':
            out[entry.name] = depth + t / 2.0
            started = True
        if started:
            depth += t
    if len(out) >= len(copper) and copper:
        return out
    n = max(len(copper) - 1, 1)
    return {layer: i * DEFAULT_BOARD_THICKNESS_MM / n for i, layer in enumerate(copper)}


class _Network:
    """Nodes and resistors of one net."""

    def __init__(self):
        self.parent: List[int] = []
        self.edges: List[dict] = []

    def node(self) -> int:
        self.parent.append(len(self.parent))
        return len(self.parent) - 1

    def find(self, a: int) -> int:
        while self.parent[a] != a:
            self.parent[a] = self.parent[self.parent[a]]
            a = self.parent[a]
        return a

    def join(self, a: int, b: int) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[rb] = ra

    def resistor(self, a: int, b: int, ohms: float, **what) -> None:
        self.edges.append(dict(what, a=a, b=b, ohms=ohms))


def _point_on_segment(px, py, s, slack: float) -> Optional[float]:
    """Parameter t in (0, 1) where (px, py) lies on segment ``s``'s
    centreline within ``slack``, strictly between its ends; else None."""
    dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
    l2 = dx * dx + dy * dy
    if l2 <= 0:
        return None
    t = ((px - s.start_x) * dx + (py - s.start_y) * dy) / l2
    if t <= 0 or t >= 1:
        return None
    if math.hypot(px - (s.start_x + t * dx), py - (s.start_y + t * dy)) > slack:
        return None
    length = math.sqrt(l2)
    if t * length <= JOIN_TOL_MM or (1 - t) * length <= JOIN_TOL_MM:
        return None
    return t


def solve_net(pcb_data, net_id: int, sources: Iterable[str], loads: Dict[str, float],
              via_plating_mm: float = DEFAULT_VIA_PLATING_MM,
              temp_rise_c: float = 10.0,
              pour_mesh_mm: float = DEFAULT_POUR_MESH_MM) -> dict:
    """Solve one net. ``sources`` are pad references ("J1.1") held at 0 V of
    drop; ``loads`` maps a pad reference to the current it draws, in A.

    Returns a dict: `loads` (each with `drop_mv`, or `unreached` when no
    copper path joins it to a source), `vias` and `segments` (each
    with `current_a`, `rating_a`, `over`), `total_current_a`,
    `max_drop_mv`, `pours` (each pour solved: layer, cells, mesh),
    `not_modelled` (what was left out of the solve) and `errors`
    (references that are not pads of the net)."""
    from check_drc import point_to_pad_distance, pad_copper_layers
    from geometry_utils import via_copper_layers
    import numpy as np

    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    heights = _layer_heights(pcb_data)
    net = pcb_data.nets.get(net_id)
    net_name = net.name if net else f"net_{net_id}"
    pads = [p for p in (getattr(pcb_data, 'pads_by_net', None) or {}).get(net_id, [])
            if getattr(p, 'pad_type', '') != 'np_thru_hole']
    by_ref = {pad_ref(p): p for p in pads}
    segs = [s for s in pcb_data.segments if s.net_id == net_id
            and not getattr(s, 'graphic', False) and s.width and s.width > 0
            and s.layer in copper]
    vias = [v for v in pcb_data.vias if v.net_id == net_id]
    result = {'net': net_name, 'loads': [], 'vias': [], 'segments': [], 'pours': [],
              'errors': [], 'not_modelled': [], 'via_plating_mm': via_plating_mm,
              'temp_rise_c': temp_rise_c, 'resistivity_ohm_m': COPPER_RESISTIVITY}
    for ref in list(sources) + list(loads):
        if ref not in by_ref:
            result['errors'].append(f"{ref} is not a pad of {net_name}")
    sources = [r for r in sources if r in by_ref]
    loads = {r: float(a) for r, a in loads.items() if r in by_ref}
    if not sources:
        result['errors'].append(f"{net_name}: no source pad")
    zones = [z for z in (getattr(pcb_data, 'zones', None) or []) if z.net_id == net_id]

    nw = _Network()
    # pads and vias: one node per copper layer each stands on, joined solid
    # for a pad (its copper is one piece) and through the barrel for a via
    pad_nodes: List[Dict[str, int]] = []
    for p in pads:
        nodes = {layer: nw.node() for layer in copper if layer in pad_copper_layers(p, copper)}
        ids = list(nodes.values())
        for other in ids[1:]:
            nw.join(ids[0], other)
        pad_nodes.append(nodes)
    via_nodes: List[Dict[str, int]] = []
    for v in vias:
        via_nodes.append({layer: nw.node() for layer in via_copper_layers(v, copper)})

    # every point a track is cut at: its own ends, and each track end, via or
    # pad centre that lands on it between its ends
    cuts: List[List[Tuple[float, int]]] = []
    ends: List[Tuple[int, int]] = []
    for s in segs:
        a, b = nw.node(), nw.node()
        ends.append((a, b))
        cuts.append([(0.0, a), (1.0, b)])
    by_layer: Dict[str, List[int]] = {}
    for i, s in enumerate(segs):
        by_layer.setdefault(s.layer, []).append(i)

    def attach(px, py, layer, node, reach=0.0):
        """Join ``node`` to every track of ``layer`` passing (px, py)."""
        for j in by_layer.get(layer, ()):
            t = segs[j]
            for (ex, ey), end in (((t.start_x, t.start_y), ends[j][0]),
                                  ((t.end_x, t.end_y), ends[j][1])):
                if math.hypot(px - ex, py - ey) <= reach + JOIN_TOL_MM:
                    nw.join(node, end)
            at = _point_on_segment(px, py, t, t.width / 2.0 + JOIN_TOL_MM)
            if at is not None:
                cuts[j].append((at, node))

    for i, s in enumerate(segs):
        for (px, py), end in (((s.start_x, s.start_y), ends[i][0]),
                              ((s.end_x, s.end_y), ends[i][1])):
            attach(px, py, s.layer, end)
    for v, nodes in zip(vias, via_nodes):
        for layer, node in nodes.items():
            attach(v.x, v.y, layer, node, reach=v.size / 2.0)
    for p, nodes in zip(pads, pad_nodes):
        for layer, node in nodes.items():
            for j in by_layer.get(layer, ()):
                t = segs[j]
                for (ex, ey), end in (((t.start_x, t.start_y), ends[j][0]),
                                      ((t.end_x, t.end_y), ends[j][1])):
                    if point_to_pad_distance(ex, ey, p) <= JOIN_TOL_MM:
                        nw.join(node, end)
            for v, vn in zip(vias, via_nodes):
                if layer in vn and point_to_pad_distance(v.x, v.y, p) <= JOIN_TOL_MM:
                    nw.join(node, vn[layer])

    # resistors
    for i, s in enumerate(segs):
        length = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        oz = stackup_copper_oz(pcb_data, s.layer)
        ohm_per_mm = COPPER_RESISTIVITY / (s.width * 1e-3 * oz * OZ_TO_METERS) * 1e-3
        pts = sorted(cuts[i], key=lambda c: c[0])
        for (t0, n0), (t1, n1) in zip(pts, pts[1:]):
            if t1 - t0 <= 0:
                nw.join(n0, n1)
                continue
            nw.resistor(n0, n1, ohm_per_mm * length * (t1 - t0), kind='segment',
                        index=i, part_mm=length * (t1 - t0))
    for k, (v, nodes) in enumerate(zip(vias, via_nodes)):
        on = [layer for layer in copper if layer in nodes]
        wall = max(min(via_plating_mm, v.drill / 2.0), 1e-6)
        area_mm2 = math.pi * (v.drill - wall) * wall if v.drill > wall else math.pi * wall * wall
        for la, lb in zip(on, on[1:]):
            h = abs(heights.get(lb, 0.0) - heights.get(la, 0.0))
            ohms = COPPER_RESISTIVITY * (h * 1e-3) / (area_mm2 * 1e-6) if h > 0 else 0.0
            if ohms <= 0:
                nw.join(nodes[la], nodes[lb])
            else:
                nw.resistor(nodes[la], nodes[lb], ohms, kind='via', index=k,
                            layers=(la, lb), area_mm2=area_mm2)

    # pours: each fill model coarsened to a mesh of sheet resistors
    meshed = 0
    if zones:
        try:
            from plane_fill_model import get_fill_models
            models = get_fill_models(pcb_data, net_id)
        except Exception as e:                                 # noqa: BLE001
            models = {}
            result['not_modelled'].append(f"copper pours of {net_name}: the fill "
                                          f"model could not be built ({e})")
        for layer, layer_models in sorted(models.items()):
            if layer not in copper:
                continue
            oz = stackup_copper_oz(pcb_data, layer)
            sheet_ohms = COPPER_RESISTIVITY / (oz * OZ_TO_METERS)
            for m in layer_models:
                k = max(1, int(round(pour_mesh_mm / m.cell)))
                fill = (m.labels > 0)
                cx, cy = fill.shape[0] // k, fill.shape[1] // k
                if cx < 1 or cy < 1:
                    continue
                # the share of each mesh cell that is copper: a cell on the
                # pour's edge conducts in proportion, so a strip keeps its
                # true width whatever the mesh
                blocks = fill[:cx * k, :cy * k].reshape(cx, k, cy, k).mean(axis=(1, 3))
                on = blocks > 0.02
                if not on.any():
                    continue
                side = m.cell * k
                ids = np.full(on.shape, -1, dtype=np.int64)
                first = len(nw.parent)
                count = int(on.sum())
                nw.parent.extend(range(first, first + count))
                ids[on] = np.arange(first, first + count)
                for a_ids, b_ids, frac_a, frac_b in (
                        (ids[:-1, :], ids[1:, :], blocks[:-1, :], blocks[1:, :]),
                        (ids[:, :-1], ids[:, 1:], blocks[:, :-1], blocks[:, 1:])):
                    both = (a_ids >= 0) & (b_ids >= 0)
                    share = (frac_a[both] + frac_b[both]) / 2.0
                    for a, b, f in zip(a_ids[both].tolist(), b_ids[both].tolist(),
                                       share.tolist()):
                        nw.resistor(a, b, sheet_ohms / f, kind='pour', index=meshed)
                xs = m.x0 + (np.arange(cx) + 0.5) * side
                ys = m.y0 + (np.arange(cy) + 0.5) * side

                def cells_within(px, py, reach, xs=xs, ys=ys, ids=ids, side=side):
                    """(node, x, y) of the mesh cells whose centre is within
                    ``reach`` of (px, py); the nearest one when none is, if
                    it is within a cell."""
                    i0 = max(int((px - reach - xs[0]) / side), 0)
                    i1 = min(int((px + reach - xs[0]) / side) + 2, len(xs))
                    j0 = max(int((py - reach - ys[0]) / side), 0)
                    j1 = min(int((py + reach - ys[0]) / side) + 2, len(ys))
                    if i0 >= i1 or j0 >= j1:
                        return []
                    gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing='ij')
                    win = ids[i0:i1, j0:j1]
                    d = np.hypot(gx - px, gy - py)
                    hit = (win >= 0) & (d <= reach)
                    if hit.any():
                        return list(zip(win[hit].tolist(), gx[hit].tolist(),
                                        gy[hit].tolist()))
                    d = np.where(win >= 0, d, np.inf)
                    if np.isfinite(d).any() and d.min() <= reach + side:
                        at = np.unravel_index(d.argmin(), d.shape)
                        return [(int(win[at]), float(gx[at]), float(gy[at]))]
                    return []

                for v, nodes in zip(vias, via_nodes):
                    if layer in nodes:
                        for c, _x, _y in cells_within(v.x, v.y, v.size / 2.0):
                            nw.join(nodes[layer], c)
                for p, nodes in zip(pads, pad_nodes):
                    if layer not in nodes:
                        continue
                    reach = math.hypot(p.size_x, p.size_y) / 2.0
                    for c, x, y in cells_within(p.global_x, p.global_y, reach):
                        if point_to_pad_distance(x, y, p) <= side / 2.0:
                            nw.join(nodes[layer], c)
                for j in by_layer.get(layer, ()):
                    t = segs[j]
                    for (ex, ey), end in (((t.start_x, t.start_y), ends[j][0]),
                                          ((t.end_x, t.end_y), ends[j][1])):
                        for c, _x, _y in cells_within(ex, ey, t.width / 2.0):
                            nw.join(end, c)
                result['pours'].append({'layer': layer, 'cells': count,
                                        'mesh_mm': round(side, 4),
                                        'sheet_mohm_per_sq': round(sheet_ohms * 1e3, 4)})
                meshed += 1
        unmeshed = len(zones) - sum(len(v) for v in models.values())
        if unmeshed > 0 and not any('fill model' in n for n in result['not_modelled']):
            result['not_modelled'].append(
                f"{unmeshed} copper pour(s) of {net_name} have no fill model and are "
                f"not solved, so a drop reported here is an upper bound")
        if meshed:
            result['not_modelled'].append(
                "thermal-relief spokes: a pad's contact with a pour is taken as solid")

    # reduce to joined nodes and solve G v = i with the sources at 0
    roots = sorted({nw.find(n) for n in range(len(nw.parent))})
    idx = {r: i for i, r in enumerate(roots)}
    n = len(roots)

    def nid(node):
        return idx[nw.find(node)]

    def pad_node(ref):
        nodes = pad_nodes[pads.index(by_ref[ref])]
        return nid(next(iter(nodes.values()))) if nodes else None

    src = {pad_node(r) for r in sources} - {None}
    adj: Dict[int, set] = {i: set() for i in range(n)}
    live = []
    for e in nw.edges:
        a, b = nid(e['a']), nid(e['b'])
        if a == b:
            continue
        live.append((a, b, e))
        adj[a].add(b)
        adj[b].add(a)
    reached, stack = set(src), list(src)
    while stack:
        for m in adj[stack.pop()]:
            if m not in reached:
                reached.add(m)
                stack.append(m)
    volts = np.zeros(n)
    unknown = sorted(reached - src)
    if unknown:
        from scipy.sparse import coo_matrix
        from scipy.sparse.linalg import spsolve
        pos = {m: i for i, m in enumerate(unknown)}
        rows, cols, vals = [], [], []
        rhs = np.zeros(len(unknown))
        for a, b, e in live:
            if a not in reached:
                continue
            c = 1.0 / e['ohms']
            for x, y in ((a, b), (b, a)):
                if x in pos:
                    rows.append(pos[x])
                    cols.append(pos[x])
                    vals.append(c)
                    if y in pos:
                        rows.append(pos[x])
                        cols.append(pos[y])
                        vals.append(-c)
        for ref, amps in loads.items():
            m = pad_node(ref)
            if m in pos:
                rhs[pos[m]] -= amps
        g = coo_matrix((vals, (rows, cols)), shape=(len(unknown), len(unknown))).tocsc()
        sol = np.atleast_1d(spsolve(g, rhs))
        for m, i in pos.items():
            volts[m] = sol[i]

    total = 0.0
    for ref, amps in sorted(loads.items()):
        m = pad_node(ref)
        row = {'pad': ref, 'current_a': amps}
        if m is None or m not in reached:
            row['unreached'] = True
        else:
            row['drop_mv'] = round(float(-volts[m]) * 1e3, 4)
            total += amps
        result['loads'].append(row)
    if any(r.get('unreached') for r in result['loads']):
        result['not_modelled'].append(
            "load(s) with no copper path to a source are not solved")

    seg_amps: Dict[int, float] = {}
    via_amps: Dict[int, float] = {}
    for a, b, e in live:
        if a not in reached:
            continue
        amps = abs(volts[a] - volts[b]) / e['ohms']
        if e['kind'] == 'pour':
            continue
        bucket = seg_amps if e['kind'] == 'segment' else via_amps
        bucket[e['index']] = max(bucket.get(e['index'], 0.0), amps)
    for i, amps in sorted(seg_amps.items()):
        s = segs[i]
        oz = stackup_copper_oz(pcb_data, s.layer)
        rating = calculate_max_current_ipc2152(s.width, oz, temp_rise_c,
                                               s.layer.startswith('In'))
        result['segments'].append({
            'layer': s.layer, 'width_mm': s.width,
            'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
            'length_mm': round(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y), 4),
            'current_a': round(float(amps), 6), 'rating_a': round(rating, 4),
            'over': bool(amps > rating + 1e-9)})
    for k, amps in sorted(via_amps.items()):
        v = vias[k]
        wall = max(min(via_plating_mm, v.drill / 2.0), 1e-6)
        area_mm2 = math.pi * (v.drill - wall) * wall if v.drill > wall else math.pi * wall * wall
        # the barrel as a conductor of that cross-section inside the board
        rating = calculate_max_current_ipc2221(area_mm2 / (OZ_TO_METERS * 1e3), 1.0,
                                               temp_rise_c, True)
        result['vias'].append({
            'at': (v.x, v.y), 'size_mm': v.size, 'drill_mm': v.drill,
            'type': getattr(v, 'via_type', 'through') or 'through',
            'layers': list(via_nodes[k]), 'current_a': round(float(amps), 6),
            'rating_a': round(rating, 4), 'over': bool(amps > rating + 1e-9)})
    drops = [r['drop_mv'] for r in result['loads'] if 'drop_mv' in r]
    result['total_current_a'] = round(total, 6)
    result['max_drop_mv'] = max(drops) if drops else None
    result['over'] = (sum(1 for r in result['segments'] if r['over'])
                      + sum(1 for r in result['vias'] if r['over']))
    return result


def read_loads(path: str) -> Dict[str, dict]:
    """The loads file: {"nets": {"<net name>": {"sources": ["J1.1", ...],
    "loads": {"U1.1": 0.5, ...}}}} (currents in A). Returns the `nets` map;
    raises ValueError naming what is wrong with it."""
    import json
    with open(path, encoding='utf-8') as fh:
        doc = json.load(fh)
    nets = doc.get('nets') if isinstance(doc, dict) else None
    if not isinstance(nets, dict) or not nets:
        raise ValueError('expected {"nets": {"<net>": {"sources": [...], "loads": {...}}}}')
    for name, spec in nets.items():
        if not isinstance(spec, dict):
            raise ValueError(f"{name}: expected an object with sources and loads")
        if not isinstance(spec.get('sources'), list) or not all(
                isinstance(s, str) for s in spec['sources']):
            raise ValueError(f'{name}: "sources" must be a list of pad references')
        loads = spec.get('loads')
        if not isinstance(loads, dict) or not all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and v >= 0
                for v in loads.values()):
            raise ValueError(f'{name}: "loads" must map a pad reference to amps >= 0')
    return nets
