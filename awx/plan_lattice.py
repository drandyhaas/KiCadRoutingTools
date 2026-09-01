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
        # foreign PADS -- the passives around a dest array (decoupling
        # caps, termination resistors) were invisible, so a berth ask
        # could land under a cap. A through pad (or NPTH drill) blocks
        # both layers; an SMD pad its own. The array's own balls fall
        # outside mark()'s TOL band by construction (gap lines run
        # between them), so this adds only true foreigners.
        for fp2 in pcb.footprints.values():
            for q in fp2.pads:
                n = nm.get(q.net_id, '')
                if n and n in skip_nets:
                    continue
                if not (x0 < q.global_x < x1 and y0 < q.global_y < y1):
                    continue
                lays = ('F.Cu', 'B.Cu') if q.drill > 0 else \
                    tuple(l_ for l_ in q.layers
                          if l_ in ('F.Cu', 'B.Cu'))
                sx = max(q.size_x, 0.05)
                sy = max(q.size_y, 0.05)
                nx = int(sx / 0.1) + 2
                ny = int(sy / 0.1) + 2
                who = 'pad:' + (n or fp2.reference)
                for ix in range(nx):
                    for iy in range(ny):
                        px_ = q.global_x - sx / 2 + ix * sx / max(nx - 1, 1)
                        py_ = q.global_y - sy / 2 + iy * sy / max(ny - 1, 1)
                        for lay in lays:
                            mark(px_, py_, lay, who)

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

    def _lv(self, lines, t):
        """Line coordinate at index t; a HALF index (midpoint exit)
        interpolates between its two lines."""
        i0 = int(t)
        return lines[i0] if t == i0 else (lines[i0] + lines[i0 + 1]) / 2

    def node_coord(self, face, node):
        i, j = node
        return self._lv(self.vl, j) if face in ('up', 'down') \
            else self._lv(self.hl, i)

    def exit_pt(self, face, node):
        i, j = node
        m = 0.35
        if face == 'up':
            return (self._lv(self.vl, j), self.hl[0] - m)
        if face == 'down':
            return (self._lv(self.vl, j), self.hl[-1] + m)
        if face == 'left':
            return (self.vl[0] - m, self._lv(self.hl, i))
        return (self.vl[-1] + m, self._lv(self.hl, i))

    def _exit_owned(self, key, net):
        own = self.exit_at.get(key)
        return own is not None and own != net and own not in self.ignore

    def reachable(self, nd, par):
        """Whether commit(net, par, nd) can walk a path -- a midpoint
        exit rides either of its two adjacent rim nodes."""
        i, j, lay = nd
        if i == int(i) and j == int(j):
            return nd in par
        if j != int(j):
            return (i, int(j), lay) in par or (i, int(j) + 1, lay) in par
        return (int(i), j, lay) in par or (int(i) + 1, j, lay) in par

    # menu: cheapest (vias, cells) to every rim node AND every rim
    # cell midpoint, per exit layer. Midpoints exist because the
    # engine legalizes teeth at SLOT_W=0.40 along a face while rim
    # nodes sit one pitch (0.65/0.8) apart -- node-only quantization
    # under-offered ~a third of real rim capacity. A midpoint is
    # 0.325 < SLOT_W from both neighbours, so it EXCLUDES foreign
    # exits at both adjacent nodes and vice versa.
    def menu(self, net, faces=('up', 'down', 'left', 'right')):
        dist, par = self.dijkstra(net)
        out = []
        for face in faces:
            rim = self.rim_nodes(face)
            horiz = face in ('up', 'down')
            for node in rim:
                for lay in ('F.Cu', 'B.Cu'):
                    i, j = node
                    if self._exit_owned((i, j, lay), net):
                        continue
                    mids = ([(i, j - 0.5, lay), (i, j + 0.5, lay)]
                            if horiz else
                            [(i - 0.5, j, lay), (i + 0.5, j, lay)])
                    if any(self._exit_owned(m, net) for m in mids):
                        continue
                    nd = (i, j, lay)
                    if nd in dist:
                        d = dist[nd]
                        out.append({'face': face, 'node': node,
                                    'layer': lay,
                                    'coord': self.node_coord(face, node),
                                    'vias': d // self.VIA_W,
                                    'cells': d % self.VIA_W,
                                    'nd': nd})
            for n1, n2 in zip(rim, rim[1:]):
                for lay in ('F.Cu', 'B.Cu'):
                    mid = ((n1[0] + n2[0]) / 2, (n1[1] + n2[1]) / 2,
                           lay)
                    cell = (('h', n1[0], min(n1[1], n2[1])) if horiz
                            else ('v', n1[1], min(n1[0], n2[0])))
                    if not self._cell_free(cell[0], cell[1], cell[2],
                                           lay, net):
                        continue
                    if self._exit_owned(mid, net) or any(
                            self._exit_owned((n[0], n[1], lay), net)
                            for n in (n1, n2)):
                        continue
                    ds = [dist[(n[0], n[1], lay)] for n in (n1, n2)
                          if (n[0], n[1], lay) in dist]
                    if not ds:
                        continue
                    d = min(ds) + self.CELL_W
                    out.append({'face': face,
                                'node': (mid[0], mid[1]),
                                'layer': lay,
                                'coord': self.node_coord(
                                    face, (mid[0], mid[1])),
                                'vias': d // self.VIA_W,
                                'cells': d % self.VIA_W,
                                'nd': mid})
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
        i, j, lay = nd
        walk, midcell = nd, None
        if i != int(i) or j != int(j):
            # midpoint exit: the path walks to an adjacent rim node,
            # then spends the rim cell to reach the tooth site
            if j != int(j):
                cands = [(i, int(j), lay), (i, int(j) + 1, lay)]
                midcell = ('h', int(i), int(j), lay)
            else:
                cands = [(int(i), j, lay), (int(i) + 1, j, lay)]
                midcell = ('v', int(j), int(i), lay)
            walk = next((c for c in cands if c in par), cands[0])
        cells, vias = self.extract(par, walk)
        if midcell is not None:
            cells = cells + [midcell]
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
