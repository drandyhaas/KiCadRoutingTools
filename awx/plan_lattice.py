"""#622 gap-lattice v2 (menu paths by Dijkstra) -- shared by
plan_global.py's verify/menus/solve.

Model: a BGA array's escape space is the lattice of GAP LINES between
ball rows/columns (outer rim lines included). Nodes are the diamond
intersections (h-line i, v-line j, layer); an edge traverses one gap
cell (capacity: ONE 0.1/0.1 track per cell per layer -- 0.65 pitch
gives 0.37mm gaps, 0.8 pitch 0.40mm, both hold exactly one); a via is
a layer-change edge at a node and physically reserves its diamond's
four incident cells on BOTH layers. Foreign copper is rasterized onto
cells first, so "achievable" is never a heuristic: it is collision
against real reservations. Far travel is allowed and simply spends
cells -- the density budget the flank actually has."""
import heapq
import math


class Lattice:
    def __init__(self, pcb, ref):
        fp = pcb.footprints[ref]
        self.xs = sorted({round(q.global_x, 2) for q in fp.pads})
        self.ys = sorted({round(q.global_y, 2) for q in fp.pads})
        px = self.xs[1] - self.xs[0]
        self.hl = ([self.ys[0] - px / 2]
                   + [(a + b) / 2 for a, b in zip(self.ys, self.ys[1:])]
                   + [self.ys[-1] + px / 2])
        self.vl = ([self.xs[0] - px / 2]
                   + [(a + b) / 2 for a, b in zip(self.xs, self.xs[1:])]
                   + [self.xs[-1] + px / 2])
        self.R = len(self.hl) - 1   # h-line index 0..R
        self.C = len(self.vl) - 1
        self.ball = {}
        for q in fp.pads:
            n = (q.net_name or '').split('/')[-1]
            if n:
                self.ball.setdefault(n, (
                    self.ys.index(round(q.global_y, 2)),
                    self.xs.index(round(q.global_x, 2))))
        self.used = {}        # cell key -> owner
        self.via_at = {}      # (i, j) -> owner
        self.exit_at = {}     # (i, j, layer) -> owner: a TOOTH is a
        #                       resource -- two stubs cannot end at the
        #                       same boundary node on one layer (the
        #                       SDQ11/SDQ12 same-node collision class)
        self.paths = {}       # net -> (cells, via nodes)
        self.ignore = set()   # owners treated as absent (eviction probe)

    # cells: ('h', i, k) spans h-line i between v-lines k,k+1;
    #        ('v', j, k) spans v-line j between h-lines k,k+1
    def _incident(self, i, j):
        out = []
        if j > 0:
            out.append(('h', i, j - 1))
        if j < self.C:
            out.append(('h', i, j))
        if i > 0:
            out.append(('v', j, i - 1))
        if i < self.R:
            out.append(('v', j, i))
        return out

    def _cell_free(self, kind, line, k, lay, net):
        own = self.used.get((kind, line, k, lay))
        return own is None or own == net or own in self.ignore

    def _node_ok(self, i, j, net):
        own = self.via_at.get((i, j))
        return own is None or own == net or own in self.ignore

    def reserve_board(self, pcb, skip_nets):
        nm = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
        x0, x1 = self.vl[0] - 0.3, self.vl[-1] + 0.3
        y0, y1 = self.hl[0] - 0.3, self.hl[-1] + 0.3
        TOL = 0.10

        def mark(x, y, lay, who):
            for i, hy in enumerate(self.hl):
                if abs(y - hy) < TOL:
                    for k in range(self.C):
                        if self.vl[k] - TOL <= x <= self.vl[k + 1] + TOL:
                            self.used.setdefault(('h', i, k, lay), who)
            for j, vx in enumerate(self.vl):
                if abs(x - vx) < TOL:
                    for k in range(self.R):
                        if self.hl[k] - TOL <= y <= self.hl[k + 1] + TOL:
                            self.used.setdefault(('v', j, k, lay), who)

        for s in pcb.segments:
            n = nm.get(s.net_id, '')
            if n in skip_nets:
                continue
            mx = (s.start_x + s.end_x) / 2
            my = (s.start_y + s.end_y) / 2
            if not (x0 < mx < x1 and y0 < my < y1):
                continue
            L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
            for t in range(int(L / 0.1) + 2):
                f = min(1.0, t * 0.1 / max(L, 1e-9))
                mark(s.start_x + f * (s.end_x - s.start_x),
                     s.start_y + f * (s.end_y - s.start_y),
                     s.layer, 'net:' + n)
        for v in pcb.vias:
            n = nm.get(v.net_id, '')
            if n in skip_nets:
                continue
            if x0 < v.x < x1 and y0 < v.y < y1:
                for lay in ('F.Cu', 'B.Cu'):
                    mark(v.x, v.y, lay, 'via:' + n)

    # ---- Dijkstra from a ball over (i, j, layer); via edge = layer
    # change at a node (requires the diamond's 4 cells free on BOTH
    # layers; reserving them is done at commit). Returns dist, parent.
    VIA_W = 1000
    CELL_W = 1

    def dijkstra(self, net, start_layer='F.Cu'):
        r, c = self.ball[net]
        starts = [(r, c), (r, c + 1), (r + 1, c), (r + 1, c + 1)]
        dist, par = {}, {}
        pq = []
        for (i, j) in starts:
            if self._node_ok(i, j, net):
                nd = (i, j, start_layer)
                dist[nd] = 0
                par[nd] = None
                heapq.heappush(pq, (0, nd))
        other = {'F.Cu': 'B.Cu', 'B.Cu': 'F.Cu'}
        while pq:
            d, nd = heapq.heappop(pq)
            if d > dist.get(nd, 1e18):
                continue
            i, j, lay = nd
            steps = []
            if j > 0 and self._cell_free('h', i, j - 1, lay, net):
                steps.append(((i, j - 1, lay), self.CELL_W))
            if j < self.C and self._cell_free('h', i, j, lay, net):
                steps.append(((i, j + 1, lay), self.CELL_W))
            if i > 0 and self._cell_free('v', j, i - 1, lay, net):
                steps.append(((i - 1, j, lay), self.CELL_W))
            if i < self.R and self._cell_free('v', j, i, lay, net):
                steps.append(((i + 1, j, lay), self.CELL_W))
            # via rule calibrated to what the engine demonstrably
            # places: the DESTINATION layer's diamond cells must be
            # free (the arrival track needs room); the source layer's
            # neighbourhood is manageable by sub-grid nudges (the
            # engine fans SA8 out on B inside a flank whose F cells
            # are all held). Commit still reserves both layers.
            if self._node_ok(i, j, net) and all(
                    self._cell_free(k_, l_, c_, other[lay], net)
                    for (k_, l_, c_) in self._incident(i, j)):
                steps.append(((i, j, other[lay]), self.VIA_W))
            for nd2, w in steps:
                i2, j2, _ = nd2
                if not self._node_ok(i2, j2, net):
                    continue
                d2 = d + w
                if d2 < dist.get(nd2, 1e18):
                    dist[nd2] = d2
                    par[nd2] = nd
                    heapq.heappush(pq, (d2, nd2))
        return dist, par

    def rim_nodes(self, face):
        if face == 'up':
            return [(0, j) for j in range(self.C + 1)]
        if face == 'down':
            return [(self.R, j) for j in range(self.C + 1)]
        if face == 'left':
            return [(i, 0) for i in range(self.R + 1)]
        return [(i, self.C) for i in range(self.R + 1)]

    def node_coord(self, face, node):
        i, j = node
        return self.vl[j] if face in ('up', 'down') else self.hl[i]

    def exit_pt(self, face, node):
        i, j = node
        m = 0.35
        if face == 'up':
            return (self.vl[j], self.hl[0] - m)
        if face == 'down':
            return (self.vl[j], self.hl[-1] + m)
        if face == 'left':
            return (self.vl[0] - m, self.hl[i])
        return (self.vl[-1] + m, self.hl[i])

    # menu: cheapest (vias, cells) to every rim node, per exit layer
    def menu(self, net, faces=('up', 'down', 'left', 'right')):
        dist, par = self.dijkstra(net)
        out = []
        for face in faces:
            for node in self.rim_nodes(face):
                for lay in ('F.Cu', 'B.Cu'):
                    own = self.exit_at.get((node[0], node[1], lay))
                    if own is not None and own != net \
                            and own not in self.ignore:
                        continue
                    nd = (node[0], node[1], lay)
                    if nd in dist:
                        d = dist[nd]
                        out.append({'face': face, 'node': node,
                                    'layer': lay,
                                    'coord': self.node_coord(face, node),
                                    'vias': d // self.VIA_W,
                                    'cells': d % self.VIA_W,
                                    'nd': nd})
        return out, par

    def extract(self, par, nd):
        cells, vias = [], []
        cur = nd
        while par.get(cur) is not None:
            p = par[cur]
            (i1, j1, l1), (i2, j2, l2) = p, cur
            if l1 != l2:
                vias.append(((i1, j1), l2))
            elif i1 == i2:
                cells.append(('h', i1, min(j1, j2), l1))
            else:
                cells.append(('v', j1, min(i1, i2), l1))
            cur = p
        return cells, vias

    def commit(self, net, par, nd):
        cells, vias = self.extract(par, nd)
        for cl in cells:
            self.used[cl] = net
        # a via consumes its diamond on the ARRIVAL layer (same rule
        # as placement); the departure layer's cells stay threadable
        # by sub-grid nudges -- the fat both-layer reservation walled
        # corners the real comb demonstrably threads
        via_layer = dict(vias)
        for (i, j), lay in via_layer.items():
            self.via_at[(i, j)] = net
            for (k_, l_, c_) in self._incident(i, j):
                self.used.setdefault((k_, l_, c_, lay), net)
        self.exit_at[nd] = net       # the tooth consumes its node
        self.paths[net] = (cells, list(via_layer))

    def restore(self, net, cells, vias, exit_nd=None):
        """Re-commit a previously ripped path verbatim (the undo of
        rip() for a net whose re-menu came back empty mid-sweep)."""
        for cl in cells:
            self.used[cl] = net
        for (i, j) in vias:
            self.via_at[(i, j)] = net
        if exit_nd is not None:
            self.exit_at[exit_nd] = net
        self.paths[net] = (cells, list(vias))

    def rip(self, net):
        self.used = {k: v for k, v in self.used.items() if v != net}
        self.via_at = {k: v for k, v in self.via_at.items() if v != net}
        self.exit_at = {k: v for k, v in self.exit_at.items()
                        if v != net}
        self.paths.pop(net, None)
