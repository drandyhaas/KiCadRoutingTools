"""DC resistance network over one net's copper (tracks, vias and pours), and its nodal solve (#1237).

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
merged into one node, and so is every pad of one named terminal.

A pour is in the network as `plane_fill_model` predicts its fill: the fill
raster is cut into square blocks of about POUR_CELL_MM (one node per block
and piece of fill connected inside it, so separate islands never merge and a
block straddling a slot does not bridge it), and two neighbouring
blocks of one component are joined by

    pour edge       (rho / t_layer) * k / n

where k is the block's side in raster cells and n the raster cell pairs of
that component across the shared side (a full block: one square of sheet
resistance). Every via, pad and track node of the net on the pour's layer
joins the block under it, and a track piece lying wholly in the fill is left
out (the fill already counts it). The fill's own approximations carry over: thermal
spokes are not modelled (a pad joins the pour solidly), and each piece of a
block is one node (without scipy, a block is one node per fill component, so a
slot narrower than a block can be bridged).

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
#: A pour's resistor grid: the block side it aims at (mm), and the most nodes
#: it builds for one net before it coarsens further.
POUR_CELL_MM = 0.2
POUR_MAX_NODES = 400_000


@dataclass(frozen=True)
class PourPiece:
    """One pour edge's copper: its layer, midpoint and centre-to-centre length."""
    layer: str
    x: float
    y: float
    length_mm: float


def pour_edges(pcb, net_id: int, nodes, offset: int, metric: str = 'ohm',
               pad_hubs=None):
    """(edges, joins, notes) adding `net_id`'s pours to a `_net_track_graph`
    built with `hub_layers=pour_layers(...)` and `nodes`.

    `edges` are (n1, n2, w, 'pour', PourPiece, None) between pour nodes
    numbered from `offset`, `w` the resistance (`metric` 'ohm', 4-neighbour)
    or the centre-to-centre length ('mm', with diagonals, for path length).
    `joins` are (graph node, pour node, 0, None, None, None): every graph node
    on a pour layer that lies on that pour's fill, and a node of `pad_hubs`
    ({graph node: pad}) to every block its pad's copper covers, so a pad
    feeds the pour over its whole area. `notes` disclose the block size."""
    import numpy as np
    from plane_fill_model import get_fill_models
    edges, joins, notes = [], [], []
    models = [(layer, m) for layer, ms in sorted(get_fill_models(pcb, net_id).items())
              for m in ms]
    if not models:
        return edges, joins, notes
    free = sum(int((m.labels > 0).sum()) for _l, m in models)
    rho_ohm_mm = __import__('plane_resistance').COPPER_RESISTIVITY * 1e3
    nxt = offset
    sizes = set()
    built = []                          # (layer, model, coarse ids, first id)
    for layer, m in models:
        k = max(1, int(round(POUR_CELL_MM / m.cell)))
        while free / (k * k) > POUR_MAX_NODES:
            k += 1
        sizes.add(round(k * m.cell, 3))
        labels = m.labels
        nx, ny = labels.shape
        cxs = m.x0 + (np.arange(nx) + 0.5) * m.cell
        cys = m.y0 + (np.arange(ny) + 0.5) * m.cell
        # Copper an earlier pour of this layer already holds is that pour's:
        # two overlapping pours are one sheet there, not two in parallel.
        mask = labels > 0
        earlier = [b for b in built if b[0] == layer]
        owner = []
        for _l, mb, cb, ob in earlier:
            bx = np.floor((cxs - mb.x0) / mb.cell).astype(np.int64)
            by = np.floor((cys - mb.y0) / mb.cell).astype(np.int64)
            okx = (bx >= 0) & (bx < cb.shape[0])
            oky = (by >= 0) & (by < cb.shape[1])
            held = np.full(labels.shape, -1, dtype=np.int64)
            held[np.ix_(okx, oky)] = cb[np.ix_(bx[okx], by[oky])]
            owner.append((held, ob))
            mask &= held < 0
        if not mask.any():
            continue
        ii, jj = np.nonzero(mask)
        coarse = np.full(labels.shape, -1, dtype=np.int64)
        coarse[ii, jj] = _block_pieces(mask, ii, jj, k, labels)
        n = int(coarse.max()) + 1
        bi = np.zeros(n, dtype=np.int64)
        bj = np.zeros(n, dtype=np.int64)
        lab = np.zeros(n, dtype=np.int64)
        bi[coarse[ii, jj]] = ii // k
        bj[coarse[ii, jj]] = jj // k
        lab[coarse[ii, jj]] = labels[ii, jj]
        side = k * m.cell
        sheet = rho_ohm_mm / _copper_mm(pcb, layer)

        def centre(c):
            return (m.x0 + (bi[c] + 0.5) * side, m.y0 + (bj[c] + 0.5) * side)

        def boundary_pairs(a, b):
            ok = (a >= 0) & (b >= 0) & (a != b)
            pa, pb = a[ok], b[ok]
            if not len(pa):
                return []
            lo, hi = np.minimum(pa, pb), np.maximum(pa, pb)
            pairs, counts = np.unique(lo * n + hi, return_counts=True)
            return [(int(p // n), int(p % n), int(c)) for p, c in zip(pairs, counts)]

        found = boundary_pairs(coarse[:-1, :].ravel(), coarse[1:, :].ravel()) + \
            boundary_pairs(coarse[:, :-1].ravel(), coarse[:, 1:].ravel())
        for a, b, cnt in found:
            (xa, ya), (xb, yb) = centre(a), centre(b)
            w = sheet * k / cnt if metric == 'ohm' else math.hypot(xb - xa, yb - ya)
            edges.append((nxt + a, nxt + b, w, 'pour',
                          PourPiece(layer, (xa + xb) / 2, (ya + yb) / 2, side), None))
        if metric == 'mm':
            # a diagonal step between blocks whose two shared neighbours are
            # both copper of the same component: path length is not Manhattan
            at = {(int(bi[c]), int(bj[c]), int(lab[c])): c for c in range(n)}
            for (x, y, l), c in at.items():
                for dy in (-1, 1):
                    d = at.get((x + 1, y + dy, l))
                    if d is not None and (x + 1, y, l) in at and (x, y + dy, l) in at:
                        edges.append((nxt + c, nxt + d, side * math.sqrt(2), 'pour',
                                      PourPiece(layer, m.x0 + (x + 1) * side,
                                                m.y0 + (y + 0.5 + dy / 2) * side,
                                                side * math.sqrt(2)), None))
        # the seam: a cell of this pour beside a cell an earlier pour holds
        for held, ob in owner:
            fill = labels > 0
            for a, b in (((slice(None, -1), slice(None)), (slice(1, None), slice(None))),
                         ((slice(None), slice(None, -1)), (slice(None), slice(1, None)))):
                for mine, theirs in ((a, b), (b, a)):
                    seam = (coarse[mine] >= 0) & fill[theirs] & (held[theirs] >= 0)
                    for pa, pb in set(zip(coarse[mine][seam].tolist(),
                                          held[theirs][seam].tolist())):
                        joins.append((nxt + pa, ob + pb, 0.0, None, None, None))
        reach = 2
        for gid, (x, y, nlayer) in enumerate(nodes):
            if nlayer != layer:
                continue
            pad = (pad_hubs or {}).get(gid)
            if pad is not None:
                under = _pad_cells(pad, cxs, cys, coarse)
                if under.size:
                    joins += [(gid, nxt + int(c), 0.0, None, None, None)
                              for c in np.unique(under)]
                    continue
            i, j = int((x - m.x0) / m.cell), int((y - m.y0) / m.cell)
            if not (0 <= i < nx and 0 <= j < ny):
                continue
            i0, j0 = max(0, i - reach), max(0, j - reach)
            win = coarse[i0:i + reach + 1, j0:j + reach + 1]
            wi, wj = np.nonzero(win >= 0)
            if wi.size:
                # the fill cell nearest the node (raster order breaks a tie)
                d2 = (cxs[i0 + wi] - x) ** 2 + (cys[j0 + wj] - y) ** 2
                c = int(win[wi[int(np.argmin(d2))], wj[int(np.argmin(d2))]])
                joins.append((gid, nxt + c, 0.0, None, None, None))
        built.append((layer, m, coarse, nxt))
        nxt += n
    notes.append('pours as plane_fill_model predicts their fill, in blocks of '
                  '%s mm; thermal spokes are not modelled'
                  % '/'.join('%g' % v for v in sorted(sizes)))
    return edges, joins, notes


def _block_pieces(mask, ii, jj, k, labels):
    """The node id of each fill cell (ii, jj): one node per block of k x k
    cells and per piece of fill that is connected INSIDE that block, so a
    block straddling a slot of one fill component is two nodes, not a bridge.
    Without scipy, one node per block and fill component."""
    import numpy as np
    try:
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
    except ImportError:
        nbj = (mask.shape[1] + k - 1) // k
        code = ((ii // k) * nbj + (jj // k)).astype(np.int64) * (int(labels.max()) + 1) \
            + labels[ii, jj]
        return np.unique(code, return_inverse=True)[1]
    idx = np.full(mask.shape, -1, dtype=np.int64)
    idx[ii, jj] = np.arange(len(ii))
    rows, cols = [], []
    same_i = (np.arange(mask.shape[0] - 1) // k) == (np.arange(1, mask.shape[0]) // k)
    same_j = (np.arange(mask.shape[1] - 1) // k) == (np.arange(1, mask.shape[1]) // k)
    for a, b, same in ((idx[:-1, :], idx[1:, :], same_i[:, None]),
                       (idx[:, :-1], idx[:, 1:], same_j[None, :])):
        ok = (a >= 0) & (b >= 0) & same
        rows.append(a[ok])
        cols.append(b[ok])
    rows, cols = np.concatenate(rows), np.concatenate(cols)
    g = coo_matrix((np.ones(len(rows), dtype=np.int8), (rows, cols)),
                   shape=(len(ii), len(ii)))
    return connected_components(g, directed=False)[1].astype(np.int64)


def _pad_cells(pad, cxs, cys, coarse):
    """The block ids of the raster cells whose centres lie on `pad`'s
    rectangle (rotated by its `rect_rotation`)."""
    import numpy as np
    hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
    rad = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
    ca, sa = math.cos(rad), math.sin(rad)
    reach = math.hypot(hx, hy)
    i = np.nonzero(np.abs(cxs - pad.global_x) <= reach)[0]
    j = np.nonzero(np.abs(cys - pad.global_y) <= reach)[0]
    if not i.size or not j.size:
        return np.zeros(0, dtype=np.int64)
    dx = cxs[i][:, None] - pad.global_x
    dy = cys[j][None, :] - pad.global_y
    inside = (np.abs(dx * ca + dy * sa) <= hx) & (np.abs(-dx * sa + dy * ca) <= hy)
    sub = coarse[i[0]:i[-1] + 1, j[0]:j[-1] + 1][inside]
    return sub[sub >= 0]


def pour_covers(pcb, net_id: int):
    """covered(layer, x, y): does the net's predicted fill on `layer` hold
    the point? The fill already counts the net's own tracks on that layer, so
    a track piece lying in it is that copper a second time."""
    from plane_fill_model import get_fill_models
    by_layer = get_fill_models(pcb, net_id)

    def covered(layer, x, y):
        for m in by_layer.get(layer, ()):
            i, j = int((x - m.x0) / m.cell), int((y - m.y0) / m.cell)
            if 0 <= i < m.nx and 0 <= j < m.ny and m.labels[i, j] > 0:
                return True
        return False
    return covered


def pour_layers(pcb, net_id: int) -> List[str]:
    """The copper layers carrying a pour of `net_id`."""
    return sorted({z.layer for z in (getattr(pcb, 'zones', None) or [])
                   if z.net_id == net_id and z.polygon})


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
    """The network of `net_id`'s tracks, vias and pours, or None when the net
    has neither a track nor a pour. `terminals` names groups of pads ({'J1.1': [pads]}); every pad of
    a group is one node (`Network.terminal_node`), so a part whose pad number
    repeats conducts through all of them.

    A via step's barrel is the stackup's length between its two layers
    (`get_via_barrel_length`), less half the copper of an end layer that is
    not one of the via's own ends, so the steps of one via (through, blind or
    buried) add up to its whole barrel. Without a stackup it is
    `board_thickness_mm` shared out by the layers' order, and the network says
    so in `assumptions`."""
    from geometry_utils import via_end_layers
    from net_queries import _net_track_graph
    from plane_resistance import COPPER_RESISTIVITY
    plating = VIA_PLATING_MM if via_plating_mm is None else float(via_plating_mm)
    rho_ohm_mm = COPPER_RESISTIVITY * 1e3
    order = list(getattr(pcb.board_info, 'copper_layers', None) or [])
    assumed_barrel = []

    stack_cu = {e.name: e.thickness for e in
                (getattr(pcb.board_info, 'stackup', None) or []) if e.layer_type == 'copper'}

    def barrel(span, via):
        l1, l2 = span
        b = pcb.get_via_barrel_length(l1, l2) if hasattr(pcb, 'get_via_barrel_length') else 0.0
        if b > 0:
            # get_via_barrel_length counts both end layers' copper in full;
            # a step end that is not one of the via's own ends is shared with
            # the next step, so each step takes half of it and the steps add
            # up to the via's whole barrel (through, blind or buried).
            ends = via_end_layers(via)
            for layer in (l1, l2):
                if layer not in ends:
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
        return rho_ohm_mm * barrel(span, obj) / (math.pi * max(obj.drill or 0.0, 1e-6) * plating)

    raw, nodes = [], []
    pads = [p for group in terminals.values() for p in group]
    poured = pour_layers(pcb, net_id)
    graph = _net_track_graph(pcb, net_id, pads, tolerance, weight=weight, edges=raw,
                             hub_layers=poured, nodes=nodes)
    if graph is None:
        return None
    _adj, pad_nodes = graph
    hubs = {h: p for p in pads for h in (pad_nodes.get(id(p)) or [])}
    pour_raw, pour_joins, pour_notes = pour_edges(pcb, net_id, nodes, len(nodes),
                                                  pad_hubs=hubs)
    if pour_notes:
        # a track piece lying wholly in its own net's fill is copper the pour
        # already carries: keep its ends (they join the pour), drop its resistor
        covered = pour_covers(pcb, net_id)

        def inside(n1, n2):
            (x1, y1, layer), (x2, y2, _l) = nodes[n1], nodes[n2]
            steps = max(1, int(math.hypot(x2 - x1, y2 - y1) / 0.04))
            return all(covered(layer, x1 + (x2 - x1) * t / steps, y1 + (y2 - y1) * t / steps)
                       for t in range(steps + 1))
        raw = [r for r in raw if not (r[3] == 'seg' and r[4].layer in poured
                                      and inside(r[0], r[1]))]
    raw += pour_raw + pour_joins

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
            if kind == 'via':
                length = barrel(span, obj)
            elif kind == 'pour':
                length = obj.length_mm
            else:
                length = w * max(obj.width, 1e-6) * _copper_mm(pcb, obj.layer) / rho_ohm_mm
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
    net.assumptions += pour_notes
    if poured and not pour_notes:
        net.assumptions.append('the fill of the pour(s) on %s could not be modelled: '
                               'they are left out, so resistances read high'
                               % ', '.join(poured))
    return net


#: Without scipy the matrix is dense: at most this many unknowns, inverted
#: once (about 30 MB); a bigger network is not solved (Solver.unavailable).
DENSE_MAX_NODES = 2000


class Solver:
    """`net` with `ground` held at 0 V, its conductance matrix factored once,
    so each set of injections costs one back-substitution (#1237: a list of
    loads is one solve per load). scipy's sparse LU when it imports; without
    scipy a dense inverse up to DENSE_MAX_NODES unknowns, and above that
    `unavailable` says why and `solve` must not be called."""

    def __init__(self, net: Network, ground: int):
        import numpy as np
        self.net, self.ground = net, ground
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
        self.seen = seen
        live = sorted(seen - {ground})
        self.pos = np.full(net.n_nodes, -1, dtype=np.int64)
        self.pos[live] = np.arange(len(live))
        self.ea = np.array([e.a for e in net.edges], dtype=np.int64)
        self.eb = np.array([e.b for e in net.edges], dtype=np.int64)
        g = 1.0 / np.array([e.ohm for e in net.edges], dtype=float) if net.edges else np.zeros(0)
        self.g = g
        self._solve = None
        self.unavailable = None
        if not live:
            return
        pa, pb = self.pos[self.ea], self.pos[self.eb]
        rows, cols, vals = [], [], []
        for x, y in ((pa, pb), (pb, pa)):
            k = x >= 0
            rows.append(x[k]); cols.append(x[k]); vals.append(g[k])          # diagonal
            k2 = k & (y >= 0)
            rows.append(x[k2]); cols.append(y[k2]); vals.append(-g[k2])
        rows, cols, vals = (np.concatenate(v) for v in (rows, cols, vals))
        size = len(live)
        try:
            from scipy.sparse import csc_matrix
            from scipy.sparse.linalg import factorized
            self._solve = factorized(csc_matrix((vals, (rows, cols)), shape=(size, size)))
        except ImportError:
            if size > DENSE_MAX_NODES:
                self.unavailable = (
                    f'scipy is not installed, and the network has {size} unknowns '
                    f'(the dense fallback takes {DENSE_MAX_NODES}): install scipy '
                    f'(pip install scipy) to judge it')
                return
            m = np.zeros((size, size))
            np.add.at(m, (rows, cols), vals)
            inv = np.linalg.inv(m)
            self._solve = lambda rhs, _inv=inv: _inv @ rhs

    def solve(self, injections: Dict[int, float]):
        """(potential, currents, unreachable): `potential` a float array (nan
        where a node is not connected to ground), `currents` the amps of each
        edge from a to b (nan where either end is not connected), for
        `injections` {node: amps, positive INTO the network}; an injection at
        an unconnected node is ignored and returned in `unreachable`."""
        import numpy as np
        unreachable = {n: a for n, a in injections.items() if n not in self.seen}
        potential = np.full(self.net.n_nodes, np.nan)
        potential[self.ground] = 0.0
        if self._solve is not None:
            rhs = np.zeros(int((self.pos >= 0).sum()))
            for n, amps in injections.items():
                if self.pos[n] >= 0:
                    rhs[self.pos[n]] += amps
            live = self.pos >= 0
            potential[live] = np.asarray(self._solve(rhs))[self.pos[live]]
        currents = (potential[self.ea] - potential[self.eb]) * self.g
        return potential, currents, unreachable


def solve(net: Network, injections: Dict[int, float], ground: int):
    """`Solver(net, ground).solve(injections)` as plain lists: (potential
    volts by node, None where unconnected; current amps by edge index, None
    where unconnected; unreachable injections)."""
    import math as _m
    potential, currents, unreachable = Solver(net, ground).solve(injections)
    return ([None if _m.isnan(v) else float(v) for v in potential],
            [None if _m.isnan(c) else float(c) for c in currents], unreachable)
