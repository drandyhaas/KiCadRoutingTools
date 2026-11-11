"""How many of a part's signal nets can leave it AT ONCE (#1817).

`reachability.pad_reachability` asks one pad, one net, one widest path: N pads
sharing one lane each read PASSABLE. `escape.py`'s lane ledger counts lanes per
face by dividing a span, and says itself that its deficit proves nothing.
This module asks the question neither can: of the part's signal nets, how many
can leave a window round it simultaneously, each by its own track, at the
board's rules. Because every net has the same place to go -- the window's rim,
or another part's copper of its own net inside the window -- it is a
single-commodity maximum flow, solved exactly
(`scipy.sparse.csgraph.maximum_flow`).

THE MODEL OVER-STATES CAPACITY, ON PURPOSE, so `deficit > 0` is a proof and
`deficit == 0` is "not proven short":

  * Free space is `reachability`'s: per layer, the clearance-adjusted room
    from foreign copper, a track's centreline legal where it is at least half
    the track width (less 3/4 of a raster step, in the generous direction).
    Space near a net's OWN copper and nowhere near another's is that net's
    alone (a pad's halo): only its own net enters it from its pads, and other
    nets reach it only to END there (it is that net's sink halo, one unit).
  * The lattice is cells of HALF a lane pitch, one unit of capacity each. Two
    track centrelines a lane pitch apart cannot both cross a cell whose
    diagonal is shorter than that, so every legal set of tracks maps to
    disjoint lattice paths; the lattice admits more, never fewer. Two cells
    are joined where their free fine cells touch (8-neighbour) across the
    border, and a via joins a cell's layers where a via fits on every layer
    (and no rule area bans it) at some point of the cell. A via's own
    footprint on the other tracks is not charged.
  * Plane nets (ground and supply names, `ignore_net_ids`) and nets with no
    copper off this part are no demand. A pad with no legal start at this
    track width is counted as escaping, and named.

What it does not say: a flow of 0 deficit is not a route (the router still
has to find it, and the window is a local question); the clearance is the
single lane clearance the ledger prices at (`escape.lane_pitch_parts`), not
each pair's class, which is the generous reading on a multi-class board.

The cut, when the deficit is positive, is named as parts: the copper nearest
each saturated lattice cell on the source side of the minimum cut
(`reachability.nearest_foreign`) -- the parts whose copper closes the pocket.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import escape
from . import reachability as reach

#: Window round the part's pad copper, mm: what reaches its rim has escaped.
DEFAULT_MARGIN_MM = 3.0
#: The finest raster step used, mm (the cell is an integer number of steps).
MAX_STEP_MM = 0.02


@dataclass
class PartFlow:
    ref: str
    demand: int
    flow: int
    nets: Tuple[str, ...] = ()
    window: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    rules: Dict[str, float] = field(default_factory=dict)
    layers: Tuple[str, ...] = ()
    cut_refs: Tuple[str, ...] = ()
    cut_cells: Tuple[Tuple[str, float, float], ...] = ()
    unstartable: Tuple[str, ...] = ()
    local: Tuple[str, ...] = ()
    ignored: Tuple[str, ...] = ()
    seconds: float = 0.0

    @property
    def deficit(self) -> int:
        return max(0, self.demand - self.flow)

    def to_dict(self):
        return {'ref': self.ref, 'demand': self.demand, 'flow': self.flow,
                'deficit': self.deficit, 'nets': list(self.nets),
                'window': [round(v, 4) for v in self.window],
                'rules': {k: round(v, 4) for k, v in self.rules.items()},
                'layers': list(self.layers), 'cut_refs': list(self.cut_refs),
                'cut_cells': [[l, round(x, 3), round(y, 3)]
                              for l, x, y in self.cut_cells],
                'unstartable': list(self.unstartable),
                'local': list(self.local), 'ignored': list(self.ignored),
                'seconds': round(self.seconds, 2),
                'basis': 'max flow on a half-lane lattice that over-states '
                         'capacity: deficit > 0 is a proof, 0 is not proven '
                         'short'}


def _stamp(owner, label, view, step, bbox, fn, *args):
    """Stamp one object with `fn` (a `reachability` rasteriser) into a small
    grid-aligned sub-array, then write `label` into `owner` where it landed."""
    h, w = owner.shape
    gx0 = max(0, int((bbox[0] - view[0]) / step) - 1)
    gx1 = min(w, int((bbox[2] - view[0]) / step) + 3)
    gy0 = max(0, int((bbox[1] - view[1]) / step) - 1)
    gy1 = min(h, int((bbox[3] - view[1]) / step) + 3)
    if gx0 >= gx1 or gy0 >= gy1:
        return None
    sub = np.zeros((gy1 - gy0, gx1 - gx0), dtype=bool)
    sv = (view[0] + gx0 * step, view[1] + gy0 * step, 0.0, 0.0)
    fn(sub, *args, sv, step)
    owner[gy0:gy1, gx0:gx1][sub] = label
    return (slice(gy0, gy1), slice(gx0, gx1), sub)


def _rect_fn(rot):
    def f(occ, cx, cy, sx, sy, view, step):
        reach._rect(occ, cx, cy, sx, sy, view, step, rot)
    return f


def _resolve_rules(pcb, pcb_file, track_width, clearance, via_diameter,
                   edge_clearance):
    import routing_defaults as defaults
    tw, clr, dr = escape.lane_pitch_parts(pcb, pcb_file, track_width,
                                          clearance, return_rules=True)
    path = pcb_file or getattr(pcb, 'source_path', None)
    via, edge = via_diameter, edge_clearance
    if path and (via is None or edge is None):
        try:
            import list_nets
            if dr is None:
                dr = list_nets.read_design_rules(path)
            if via is None:
                via = list_nets.board_floor(
                    path, 'via_diameter', None,
                    getattr(defaults, 'VIA_SIZE', 0.5), dr)[0]
            if edge is None:
                edge = list_nets.board_floor_knobs(path, design_rules=dr)[1]
        except Exception:                                   # noqa: BLE001
            pass
    if not via:
        via = getattr(defaults, 'VIA_SIZE', 0.5)
    if edge is None:
        edge = getattr(defaults, 'BOARD_EDGE_CLEARANCE', 0.5)
    return float(tw), float(clr), float(via), float(edge)


def demand_nets(pcb, ref, ignore_net_ids=()) -> Tuple[List[int], List[int]]:
    """`(demand, ignored)`: the part's nets that must leave it -- a net with
    copper on another part, not a plane net -- and the ones dropped as plane
    nets (a ground or supply name, or `ignore_net_ids`)."""
    from net_queries import is_ground_net_name, is_power_net_name
    from kicad_parser import non_aperture_pads
    fp = pcb.footprints[ref]
    skip = set(ignore_net_ids or ())
    elsewhere = set()
    for r, other in pcb.footprints.items():
        if r == ref:
            continue
        for p in other.pads:
            if p.net_id:
                elsewhere.add(p.net_id)
    demand, ignored = [], []
    for p in non_aperture_pads(fp):
        n = p.net_id
        if not n or n in demand or n in ignored:
            continue
        name = pcb.nets[n].name if n in pcb.nets else ''
        if (n in skip or is_ground_net_name(name) or is_power_net_name(name)):
            ignored.append(n)
            continue
        if n in elsewhere:
            demand.append(n)
    return sorted(demand), sorted(ignored)


def part_flow(pcb, ref, pcb_file=None, *, margin_mm: float = DEFAULT_MARGIN_MM,
              track_width: Optional[float] = None,
              clearance: Optional[float] = None,
              via_diameter: Optional[float] = None,
              edge_clearance: Optional[float] = None,
              ignore_net_ids: Sequence[int] = (),
              layers: Optional[Sequence[str]] = None) -> PartFlow:
    """How many of `ref`'s signal nets leave a window `margin_mm` round its
    pads at once, at the rules `escape_ledger` prices lanes at. See the
    module docstring for what the number proves."""
    import time
    from scipy import ndimage
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import maximum_flow
    from kicad_parser import pad_drill_capsule
    t0 = time.time()
    reach._require_scipy()
    if ref not in pcb.footprints:
        raise KeyError(ref)
    tw, clr, via, edge_c = _resolve_rules(pcb, pcb_file, track_width,
                                          clearance, via_diameter,
                                          edge_clearance)
    lane = tw + clr
    cell = lane / 2.0
    k = max(1, int(math.ceil(cell / MAX_STEP_MM - 1e-9)))
    step = cell / k
    tol = 0.75 * step
    layers = tuple(layers or pcb.board_info.copper_layers or ('F.Cu', 'B.Cu'))
    L = len(layers)
    board, _ns, _nv = reach.without_routed_copper(pcb)
    fp = board.footprints[ref]

    demand, ignored = demand_nets(board, ref, ignore_net_ids)
    names = {n: (board.nets[n].name if n in board.nets else str(n))
             for n in demand + ignored}
    res = PartFlow(ref=ref, demand=len(demand), flow=0,
                   nets=tuple(names[n] for n in demand), layers=layers,
                   ignored=tuple(names[n] for n in ignored),
                   rules={'track_mm': tw, 'clearance_mm': clr, 'via_mm': via,
                          'edge_clearance_mm': edge_c, 'lane_mm': lane,
                          'cell_mm': cell, 'step_mm': step,
                          'margin_mm': margin_mm})
    if not demand:
        res.seconds = time.time() - t0
        return res

    # -- the window, on a grid of whole lattice cells ----------------------
    xs0, ys0, xs1, ys1 = [], [], [], []
    for p in fp.pads:
        hx, hy = max(p.size_x, p.size_y) / 2.0, max(p.size_x, p.size_y) / 2.0
        xs0.append(p.global_x - hx)
        ys0.append(p.global_y - hy)
        xs1.append(p.global_x + hx)
        ys1.append(p.global_y + hy)
    x0, y0 = min(xs0) - margin_mm, min(ys0) - margin_mm
    ncx = int(math.ceil((max(xs1) + margin_mm - x0) / cell))
    ncy = int(math.ceil((max(ys1) + margin_mm - y0) / cell))
    view = (x0, y0, x0 + ncx * cell, y0 + ncy * cell)
    res.window = view
    W, H = ncx * k, ncy * k

    def in_view(bb, pad=0.0):
        return not (bb[2] < view[0] - pad or bb[0] > view[2] + pad
                    or bb[3] < view[1] - pad or bb[1] > view[3] + pad)

    # -- rasterise: per layer, which net owns each fine cell ----------------
    # label 0 empty, 1 copper of no net / an obstacle to every net, n + 1
    # net n's copper. `own` marks this part's own pads.
    FOREIGN = 1
    owner = [np.zeros((H, W), dtype=np.int32) for _ in layers]
    own = [np.zeros((H, W), dtype=bool) for _ in layers]
    edge_occ = np.zeros((H, W), dtype=bool)
    via_occ = np.zeros((H, W), dtype=bool)
    for r, f in board.footprints.items():
        for p in f.pads:
            hr = math.hypot(p.size_x, p.size_y) / 2.0
            bb = (p.global_x - hr, p.global_y - hr, p.global_x + hr,
                  p.global_y + hr)
            if not in_view(bb, clr + via):
                continue
            if getattr(p, 'pad_type', '') == 'np_thru_hole':
                if (p.drill or 0) > 0:
                    a, b, rr = pad_drill_capsule(p)
                    hb = (min(a[0], b[0]) - rr, min(a[1], b[1]) - rr,
                          max(a[0], b[0]) + rr, max(a[1], b[1]) + rr)
                    for li in range(L):
                        _stamp(owner[li], FOREIGN, view, step, hb,
                               reach._capsule, a[0], a[1], b[0], b[1], rr)
                continue
            for li, lay in enumerate(layers):
                if not reach._pad_on_layer(p, lay):
                    continue
                got = _stamp(owner[li], (p.net_id or 0) + 1, view, step, bb,
                             _rect_fn(getattr(p, 'rect_rotation', 0.0) or 0.0),
                             p.global_x, p.global_y, p.size_x, p.size_y)
                if got is not None and r == ref:
                    own[li][got[0], got[1]] |= got[2]
    for s in board.segments:
        if s.layer not in layers:
            continue
        rr = s.width / 2.0
        bb = (min(s.start_x, s.end_x) - rr, min(s.start_y, s.end_y) - rr,
              max(s.start_x, s.end_x) + rr, max(s.start_y, s.end_y) + rr)
        if in_view(bb, clr + via):
            _stamp(owner[layers.index(s.layer)], (s.net_id or 0) + 1, view,
                   step, bb, reach._capsule, s.start_x, s.start_y, s.end_x,
                   s.end_y, rr)
    for v in board.vias:
        rr = v.size / 2.0
        bb = (v.x - rr, v.y - rr, v.x + rr, v.y + rr)
        if in_view(bb, clr + via):
            for li in range(L):
                _stamp(owner[li], (v.net_id or 0) + 1, view, step, bb,
                       reach._capsule, v.x, v.y, v.x, v.y, rr)
    for (ax, ay), (bx, by) in reach.outline_edges(board):
        bb = (min(ax, bx), min(ay, by), max(ax, bx), max(ay, by))
        if in_view(bb, edge_c + via):
            reach._capsule(edge_occ, ax, ay, bx, by, step, view, step)
    for li, lay in enumerate(layers):
        for poly, holes, tracks_ok, vias_ok in reach.rule_area_keepouts(
                board, lay):
            if not tracks_ok:
                m = np.zeros((H, W), dtype=bool)
                reach._region(m, poly, holes, view, step)
                owner[li][m] = FOREIGN
            if not vias_ok:
                reach._region(via_occ, poly, holes, view, step)

    def edt(occ):
        if not occ.any():
            return np.full(occ.shape, np.inf)
        return ndimage.distance_transform_edt(~occ) * step

    # room from the outline (at the edge clearance) and the via ban
    d_edge = edt(edge_occ) - edge_c
    via_ban = (edt(via_occ) - clr) < via / 2.0 - tol if via_occ.any() else \
        np.zeros((H, W), dtype=bool)
    D_all = []
    for li in range(L):
        occ = owner[li] != 0
        D = np.minimum(edt(occ) - clr, d_edge)
        D[occ] = -1.0
        D_all.append(D)
    free_all = [D >= tw / 2.0 - tol for D in D_all]
    via_all = np.logical_and.reduce([D >= via / 2.0 - tol for D in D_all]) \
        & ~via_ban

    # -- each net's own halo: room from every copper but its own -----------
    nidx = {n: i for i, n in enumerate(demand)}
    cls = [np.where(fa, 1, 0).astype(np.int32) for fa in free_all]
    via_any = via_all.copy()
    reach_cells = int(math.ceil((clr + max(tw, via) / 2.0) / step)) + 3
    local, has_src = set(), set()
    for n in demand:
        lab = n + 1
        i = nidx[n]
        D_n = []
        boxes = []
        for li in range(L):
            m = owner[li] == lab
            if m.any():
                ys, xs = np.nonzero(m)
                boxes.append((ys.min(), ys.max(), xs.min(), xs.max()))
        if not boxes:
            continue
        # one box over the net's copper in the window: halos live within
        # `reach_cells` of it, and their room is measured within twice that
        by0 = max(0, min(b[0] for b in boxes) - 2 * reach_cells)
        by1 = min(H, max(b[1] for b in boxes) + 2 * reach_cells + 1)
        bx0 = max(0, min(b[2] for b in boxes) - 2 * reach_cells)
        bx1 = min(W, max(b[3] for b in boxes) + 2 * reach_cells + 1)
        sl = (slice(by0, by1), slice(bx0, bx1))
        for li in range(L):
            o = owner[li][sl]
            occ = (o != 0) & (o != lab)
            D = np.minimum(edt(occ) - clr, d_edge[sl])
            D[occ] = -1.0
            D_n.append(D)
        via_n = np.logical_and.reduce([D >= via / 2.0 - tol for D in D_n]) \
            & ~via_ban[sl]
        near = np.zeros(D_n[0].shape, dtype=bool)
        for li in range(L):
            near |= owner[li][sl] == lab
        near = ndimage.binary_dilation(near, iterations=reach_cells)
        via_n &= near
        via_any[sl] |= via_n
        for li in range(L):
            priv = (D_n[li] >= tw / 2.0 - tol) & ~free_all[li][sl] & near
            if not priv.any():
                continue
            lbl, nc = ndimage.label(priv, structure=np.ones((3, 3)))
            mine = (owner[li][sl] == lab)
            src = ndimage.binary_dilation(mine & own[li][sl], iterations=2)
            snk = ndimage.binary_dilation(mine & ~own[li][sl], iterations=2)
            hit_src = set(np.unique(lbl[src & (lbl > 0)]).tolist())
            hit_snk = set(np.unique(lbl[snk & (lbl > 0)]).tolist())
            c = cls[li][sl]
            for comp in range(1, nc + 1):
                a, b = comp in hit_src, comp in hit_snk
                if a and b:
                    local.add(n)
                if a:
                    c[lbl == comp] = 2 + 2 * i
                    has_src.add(n)
                elif b:
                    c[lbl == comp] = 3 + 2 * i

    # -- the lattice graph ---------------------------------------------------
    ncell = ncx * ncy
    nclass = 2 + 2 * len(demand)
    cy_of = (np.arange(H) // k)[:, None]
    cx_of = (np.arange(W) // k)[None, :]
    cid = (cy_of * ncx + cx_of).astype(np.int64)
    keys = []
    for li in range(L):
        m = cls[li] > 0
        keys.append(np.where(m, (li * ncell + cid) * nclass + cls[li], -1))
    allk = np.unique(np.concatenate([kk[kk >= 0] for kk in keys]))
    N = len(allk)
    node = []
    for kk in keys:
        idx = np.full(kk.shape, -1, dtype=np.int64)
        m = kk >= 0
        idx[m] = np.searchsorted(allk, kk[m])
        node.append(idx)
    ncls = (allk % nclass).astype(np.int64)
    ncell_of = (allk // nclass) % ncell
    nlay = allk // nclass // ncell

    def allowed(cu, cv):
        """Which class may hand flow to which: shared to shared, a net's
        source halo out to shared or its own halos, shared (or its source
        halo) into its sink halo, never across nets."""
        same = (cu >= 2) & (cv >= 2) & (((cu - 2) // 2) == ((cv - 2) // 2))
        su, sv = (cu >= 2) & ((cu - 2) % 2 == 0), (cv >= 2) & ((cv - 2) % 2 == 0)
        ku, kv = (cu >= 2) & ((cu - 2) % 2 == 1), (cv >= 2) & ((cv - 2) % 2 == 1)
        return (((cu == 1) & (cv == 1)) | (su & (cv == 1)) | (su & sv & same)
                | ((cu == 1) & kv) | (ku & kv & same) | (su & kv & same))

    eu, ev = [], []
    for li in range(L):
        nd = node[li]
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            a = nd[0:H - dy, max(0, -dx):W - max(0, dx)]
            b = nd[dy:H, max(0, dx):W - max(0, -dx)]
            m = (a >= 0) & (b >= 0) & (a != b)
            u, v = a[m], b[m]
            eu += [u, v]
            ev += [v, u]
    vy, vx = np.nonzero(via_any)
    for la in range(L):
        for lb in range(L):
            if la == lb:
                continue
            u, v = node[la][vy, vx], node[lb][vy, vx]
            m = (u >= 0) & (v >= 0)
            eu.append(u[m])
            ev.append(v[m])
    eu = np.concatenate(eu) if eu else np.zeros(0, dtype=np.int64)
    ev = np.concatenate(ev) if ev else np.zeros(0, dtype=np.int64)
    ok = allowed(ncls[eu], ncls[ev])
    pairs = np.unique(np.stack([eu[ok], ev[ok]], axis=1), axis=0) \
        if ok.any() else np.zeros((0, 2), dtype=np.int64)

    SRC, SNK = 2 * N, 2 * N + 1
    S0, T0 = 2 * N + 2, 2 * N + 2 + len(demand)
    nV = T0 + len(demand)
    rows = [np.arange(N) * 2, pairs[:, 0] * 2 + 1]
    cols = [np.arange(N) * 2 + 1, pairs[:, 1] * 2]
    ncy_of, ncx_of = ncell_of // ncx, ncell_of % ncx
    rim = (ncy_of == 0) | (ncy_of == ncy - 1) | (ncx_of == 0) | (ncx_of == ncx - 1)
    out_ok = rim & ((ncls == 1) | ((ncls >= 2) & ((ncls - 2) % 2 == 0)))
    rows.append(np.nonzero(out_ok)[0] * 2 + 1)
    cols.append(np.full(int(out_ok.sum()), SNK))
    unstartable = []
    for n in demand:
        i = nidx[n]
        rows.append(np.array([SRC]))
        cols.append(np.array([S0 + i]))
        rows.append(np.array([T0 + i]))
        cols.append(np.array([SNK]))
        srcs = np.nonzero(ncls == 2 + 2 * i)[0]
        snks = np.nonzero(ncls == 3 + 2 * i)[0]
        rows.append(np.full(len(srcs), S0 + i))
        cols.append(srcs * 2)
        rows.append(snks * 2 + 1)
        cols.append(np.full(len(snks), T0 + i))
        if n in local:
            rows.append(np.array([S0 + i]))
            cols.append(np.array([T0 + i]))
        elif not len(srcs):
            unstartable.append(n)
            rows.append(np.array([S0 + i]))
            cols.append(np.array([SNK]))
    r_ = np.concatenate(rows).astype(np.int32)
    c_ = np.concatenate(cols).astype(np.int32)
    rc = np.unique(np.stack([r_, c_], axis=1), axis=0)
    g = csr_matrix((np.ones(len(rc), dtype=np.int32), (rc[:, 0], rc[:, 1])),
                   shape=(nV, nV))
    mf = maximum_flow(g, SRC, SNK)
    res.flow = int(mf.flow_value)
    res.unstartable = tuple(names[n] for n in unstartable)
    res.local = tuple(names[n] for n in sorted(local))

    if res.deficit > 0:
        # the minimum cut: what the source still reaches in the residual
        F = getattr(mf, 'flow', None)
        if F is None:
            F = mf.residual
        F = F.tocsr()
        cap = g.tocoo()
        fl = np.asarray(F[cap.row, cap.col]).ravel()
        fwd = cap.data - fl > 0
        rr = np.concatenate([cap.row[fwd], cap.col[fl > 0]])
        cc = np.concatenate([cap.col[fwd], cap.row[fl > 0]])
        resid = csr_matrix((np.ones(len(rr), dtype=np.int8), (rr, cc)),
                           shape=(nV, nV))
        from scipy.sparse.csgraph import breadth_first_order
        seen = np.zeros(nV, dtype=bool)
        seen[breadth_first_order(resid, SRC, return_predecessors=False)] = True
        cut = np.nonzero(seen[np.arange(N) * 2] & ~seen[np.arange(N) * 2 + 1])[0]
        cells = []
        refs: Dict[str, int] = {}
        for nd in cut[:200]:
            li = int(nlay[nd])
            cxy = int(ncell_of[nd])
            x = view[0] + (cxy % ncx + 0.5) * cell
            y = view[1] + (cxy // ncx + 0.5) * cell
            cells.append((layers[li], x, y))
            for e in reach.nearest_foreign(board, x, y, None, (layers[li],)):
                if e.get('ref') and e['ref'] != ref:
                    refs[e['ref']] = refs.get(e['ref'], 0) + 1
        res.cut_cells = tuple(cells)
        res.cut_refs = tuple(sorted(refs, key=lambda r: (-refs[r], r)))
    res.seconds = time.time() - t0
    return res
