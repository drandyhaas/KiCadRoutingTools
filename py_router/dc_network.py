"""DC resistance network over one net's TRACK copper, and its nodal solve (#1237).

The graph is `net_queries._net_track_graph` -- the joining rules length
matching already uses (T-junctions, landings anywhere in a via or pad, via
barrels between the layers they span) -- with each copper edge weighed as a
resistance instead of a length:

    segment piece   rho * L / (width * t_layer)
    via step        rho * L_barrel / (pi * drill * t_plating)

`t_layer` is the stackup's copper thickness for the layer
(plane_resistance.stackup_copper_oz, 1 oz when the board has no stackup),
`t_plating` is VIA_PLATING_MM unless the caller passes one, `L_barrel` is the
stackup's barrel length between the step's two layers (or the board
thickness shared out by layer order, disclosed) and rho is copper at 20 C
(plane_resistance.COPPER_RESISTIVITY). Joins inside a pad or a via are
merged into one node, and so is every pad of one named terminal. Zone/pour copper is NOT in the network: a
net carried partly by a pour reads more resistive than it is, and a load
joined only through a pour is unreachable.

`solve` injects currents at nodes, holds one node at 0 V, and returns every
node's potential and every edge's current. scipy's sparse solver is used when
it imports, numpy's dense one otherwise.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

#: IPC-6012 Class 2 minimum average hole-wall copper (20 um). The plating is
#: not in the board file, so the network assumes it; a spec can override it.
VIA_PLATING_MM = 0.020
#: An edge at or below this resistance (ohm) joins its two nodes into one.
_SHORT_OHM = 1e-12


@dataclass
class Edge:
    a: int
    b: int
    ohm: float
    kind: str          # 'seg' or 'via'
    obj: object
    length_mm: float


@dataclass
class Network:
    net_id: int
    edges: List[Edge] = field(default_factory=list)
    terminal_node: Dict[str, int] = field(default_factory=dict)  # name -> node
    n_nodes: int = 0
    assumptions: List[str] = field(default_factory=list)


def _copper_mm(pcb, layer: str) -> float:
    from plane_resistance import stackup_copper_oz, OZ_TO_METERS
    return stackup_copper_oz(pcb, layer) * OZ_TO_METERS * 1e3


def build(pcb, net_id: int, terminals: Dict[str, list],
          via_plating_mm: Optional[float] = None,
          board_thickness_mm: Optional[float] = None,
          tolerance: float = 0.02) -> Optional[Network]:
    """The network of `net_id`'s tracks, or None when the net has no track
    copper. `terminals` names groups of pads ({'J1.1': [pads]}); every pad of
    a group is one node (`Network.terminal_node`), so a part whose pad number
    repeats conducts through all of them.

    A via step's barrel is the stackup's length between its two layers
    (`get_via_barrel_length`), less half of an inner end layer's copper, so
    the steps of one via add up to its whole barrel. Without a stackup it is
    `board_thickness_mm` shared out by the layers' order, and the network says
    so in `assumptions`."""
    from net_queries import _net_track_graph
    from plane_resistance import COPPER_RESISTIVITY
    plating = VIA_PLATING_MM if via_plating_mm is None else float(via_plating_mm)
    rho_ohm_mm = COPPER_RESISTIVITY * 1e3
    order = list(getattr(pcb.board_info, 'copper_layers', None) or [])
    assumed_barrel = []

    stack_cu = {e.name: e.thickness for e in
                (getattr(pcb.board_info, 'stackup', None) or []) if e.layer_type == 'copper'}

    def barrel(span):
        l1, l2 = span
        b = pcb.get_via_barrel_length(l1, l2) if hasattr(pcb, 'get_via_barrel_length') else 0.0
        if b > 0:
            # get_via_barrel_length counts both end layers' copper in full;
            # an INNER end is shared with the next step, so each step takes
            # half of it and the steps add up to the F.Cu-B.Cu barrel.
            for layer in (l1, l2):
                if order and layer not in (order[0], order[-1]):
                    b -= (stack_cu.get(layer) or 0.0) / 2.0
            return b
        thick = board_thickness_mm or 0.0
        if thick > 0 and l1 in order and l2 in order and len(order) > 1:
            assumed_barrel.append(span)
            return thick * abs(order.index(l1) - order.index(l2)) / (len(order) - 1)
        return 0.0

    def weight(kind, obj, length, span):
        if kind == 'seg':
            return rho_ohm_mm * length / (max(obj.width, 1e-6) * _copper_mm(pcb, obj.layer))
        return rho_ohm_mm * barrel(span) / (math.pi * max(obj.drill or 0.0, 1e-6) * plating)

    raw = []
    pads = [p for group in terminals.values() for p in group]
    graph = _net_track_graph(pcb, net_id, pads, tolerance, weight=weight, edges=raw)
    if graph is None:
        return None
    _adj, pad_nodes = graph

    parent = {}

    def find(n):
        parent.setdefault(n, n)
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    def union(x, y):
        rx, ry = find(x), find(y)
        if rx != ry:
            parent[ry] = rx

    for n1, n2, w, kind, _obj, _span in raw:
        find(n1)
        find(n2)
        if kind is None or (kind == 'seg' and w <= _SHORT_OHM):
            union(n1, n2)
    for group in terminals.values():
        hubs = [h for p in group for h in (pad_nodes.get(id(p)) or [])]
        for h in hubs[1:]:
            union(hubs[0], h)        # a terminal's copper conducts freely

    index = {}

    def node(n):
        r = find(n)
        if r not in index:
            index[r] = len(index)
        return index[r]

    net = Network(net_id=net_id)
    for n1, n2, w, kind, obj, span in raw:
        if kind is None or (kind == 'seg' and w <= _SHORT_OHM):
            continue
        a, b = node(n1), node(n2)
        if a != b:
            length = barrel(span) if kind == 'via' else w * max(obj.width, 1e-6) * \
                _copper_mm(pcb, obj.layer) / rho_ohm_mm
            net.edges.append(Edge(a, b, max(w, _SHORT_OHM * 10), kind, obj, length))
    for name, group in terminals.items():
        hubs = [h for p in group for h in (pad_nodes.get(id(p)) or [])]
        if hubs:
            net.terminal_node[name] = node(hubs[0])
    net.n_nodes = len(index)
    if assumed_barrel:
        net.assumptions.append('the stackup does not give these via barrels: the '
                               'board thickness %.3f mm is shared out by layer '
                               'order' % (board_thickness_mm or 0.0))
    stack = getattr(getattr(pcb, 'board_info', None), 'stackup', None) or []
    if not stack:
        net.assumptions.append('the board has no stackup: copper is taken as 1 oz')
    net.assumptions.append('via plating %.3f mm' % plating)
    if any(z.net_id == net_id for z in (getattr(pcb, 'zones', None) or [])):
        net.assumptions.append('the net owns a pour, which this track network '
                               'leaves out: resistances read high')
    return net


def solve(net: Network, injections: Dict[int, float], ground: int):
    """(potential volts by node, current amps by edge index) for currents
    `injections` {node: amps, positive INTO the network} with `ground` at 0 V.
    Nodes not connected to `ground` get None; an injection there is ignored
    and returned in the third element."""
    adj = {i: [] for i in range(net.n_nodes)}
    for e in net.edges:
        adj[e.a].append(e.b)
        adj[e.b].append(e.a)
    seen, stack = {ground}, [ground]
    while stack:
        n = stack.pop()
        for m in adj[n]:
            if m not in seen:
                seen.add(m)
                stack.append(m)
    live = sorted(seen - {ground})
    pos = {n: i for i, n in enumerate(live)}
    unreachable = {n: a for n, a in injections.items() if n not in seen}
    potential = {n: None for n in range(net.n_nodes)}
    potential[ground] = 0.0
    if live:
        rows, cols, vals = [], [], []
        diag = [0.0] * len(live)
        for e in net.edges:
            g = 1.0 / e.ohm
            for x, y in ((e.a, e.b), (e.b, e.a)):
                if x in pos:
                    diag[pos[x]] += g
                    if y in pos:
                        rows.append(pos[x]); cols.append(pos[y]); vals.append(-g)
        rows += list(range(len(live))); cols += list(range(len(live))); vals += diag
        rhs = [0.0] * len(live)
        for n, amps in injections.items():
            if n in pos:
                rhs[pos[n]] += amps
        try:
            from scipy.sparse import csr_matrix
            from scipy.sparse.linalg import spsolve
            v = spsolve(csr_matrix((vals, (rows, cols)), shape=(len(live), len(live))), rhs)
        except ImportError:
            import numpy as np
            m = np.zeros((len(live), len(live)))
            for r, c, x in zip(rows, cols, vals):
                m[r, c] += x
            v = np.linalg.solve(m, np.array(rhs))
        for n, i in pos.items():
            potential[n] = float(v[i])
    currents = []
    for e in net.edges:
        va, vb = potential[e.a], potential[e.b]
        currents.append(None if va is None or vb is None else (va - vb) / e.ohm)
    return potential, currents, unreachable
