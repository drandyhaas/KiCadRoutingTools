#!/usr/bin/env python3
"""Topological string extraction (v1) for the #622 bus experiment.

For each requested net: string = polyline from the source tooth FREE END
(endpoint census of its stub) to the target ball center. Initialized as
the straight chord, then relaxed to the taut representative of that
homotopy class (elastic band: Laplacian contraction + push-out-of-
obstacles + shortcut passes). Obstacles: foreign pads (full routing
margin outside the target field, bare-copper margin inside so the ball
gaps stay threadable), foreign F.Cu segments (layer-conditional; B.Cu
copper does not constrain a nominal-F string), and all foreign vias.

Outputs: crossing matrix (straight vs taut), per-string lengths and
hugged obstacles, a strings JSON, and an Eco2.User overlay board for
render_eco.py.
"""
import argparse
import json
import math
import os
import shutil
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

TRACK = 0.127
CLEAR = 0.1
MARGIN_OUT = CLEAR + TRACK / 2      # routing margin outside the field
MARGIN_IN = 0.06                     # bare-copper margin inside the field
FREEZE = 0.35                        # no pushes this close to an endpoint
STEP = 0.12                          # densify step (mm)


def d2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def seg_pt_dist(a, b, p):
    ax, ay = a
    bx, by = b
    px_, py_ = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px_ - ax, py_ - ay)
    t = max(0.0, min(1.0, ((px_ - ax) * dx + (py_ - ay) * dy) / L2))
    return math.hypot(px_ - (ax + t * dx), py_ - (ay + t * dy))


def seg_seg_dist(a, b, c, e):
    # min distance between segments ab and ce
    if seg_x(a, b, c, e, lo=0.0, hi=1.0):
        return 0.0
    return min(seg_pt_dist(a, b, c), seg_pt_dist(a, b, e),
               seg_pt_dist(c, e, a), seg_pt_dist(c, e, b))


def seg_x(a, b, c, e, lo=0.0, hi=1.0):
    # transversal intersection of ab x ce with params in (lo,hi)
    d1x, d1y = b[0] - a[0], b[1] - a[1]
    d2x, d2y = e[0] - c[0], e[1] - c[1]
    den = d1x * d2y - d1y * d2x
    if abs(den) < 1e-12:
        return None
    t = ((c[0] - a[0]) * d2y - (c[1] - a[1]) * d2x) / den
    u = ((c[0] - a[0]) * d1y - (c[1] - a[1]) * d1x) / den
    if lo < t < hi and lo < u < hi:
        return (a[0] + t * d1x, a[1] + t * d1y)
    return None


class Obstacles:
    """Discs [(x,y,r,name)] + capsules [(a,b,r,name)] with a coarse
    spatial hash for point queries."""

    def __init__(self):
        self.discs = []
        self.caps = []
        self._grid = {}
        self.cell = 1.0

    def add_disc(self, x, y, r, name):
        self.discs.append((x, y, r, name))

    def add_cap(self, a, b, r, name):
        self.caps.append((a, b, r, name))

    def build(self):
        for i, (x, y, r, _n) in enumerate(self.discs):
            for gx in range(int((x - r - 0.3) / self.cell),
                            int((x + r + 0.3) / self.cell) + 1):
                for gy in range(int((y - r - 0.3) / self.cell),
                                int((y + r + 0.3) / self.cell) + 1):
                    self._grid.setdefault((gx, gy), []).append(i)
        # capsules hashed by their bounding box too: a point query used
        # to walk EVERY capsule (thousands of foreign fanout stubs on a
        # BGA board), which made the spine relaxation cost minutes
        self._cgrid = {}
        for i, (a, b, r, _n) in enumerate(self.caps):
            x0, x1 = min(a[0], b[0]) - r - 0.3, max(a[0], b[0]) + r + 0.3
            y0, y1 = min(a[1], b[1]) - r - 0.3, max(a[1], b[1]) + r + 0.3
            for gx in range(int(x0 / self.cell), int(x1 / self.cell) + 1):
                for gy in range(int(y0 / self.cell), int(y1 / self.cell) + 1):
                    self._cgrid.setdefault((gx, gy), []).append(i)
        # merged 3x3 neighbourhoods, precomputed ONCE. Point queries
        # used to re-concatenate nine cell lists (discs) and build a
        # set (caps) PER CALL -- 760k point_violation calls put 23 s
        # of the K28 braid's 79 s profile in that churn. The disc
        # sequence reproduces the old per-call concatenation order
        # exactly (same dx/dy nesting), so disc results are
        # bit-identical; caps become an order-stable dedupe (the old
        # set had no defined order for equal-depth ties).
        self._near_d = {}
        self._near_c = {}
        centers = {(gx + dx_, gy + dy_)
                   for k in (set(self._grid) | set(self._cgrid))
                   for gx, gy in (k,)
                   for dx_ in (-1, 0, 1) for dy_ in (-1, 0, 1)}
        for (gx, gy) in centers:
            dd, cc = [], []
            for dx_ in (-1, 0, 1):
                for dy_ in (-1, 0, 1):
                    dd.extend(self._grid.get((gx + dx_, gy + dy_), ()))
                    cc.extend(self._cgrid.get((gx + dx_, gy + dy_), ()))
            if dd:
                self._near_d[(gx, gy)] = tuple(dd)
            if cc:
                self._near_c[(gx, gy)] = tuple(dict.fromkeys(cc))
        # flat per-cell geometry packs for point_violation: the same
        # candidates in the same order, but as plain floats with the
        # capsule direction and L2 precomputed -- the 1.3M-call hot
        # loop then runs without per-candidate list indexing, tuple
        # unpacking or seg_pt_dist call frames (measured: numpy per
        # call is 3x SLOWER at these ~14-candidate sizes, so the win
        # is fewer interpreter frames, not vectorisation; arithmetic
        # and order are bit-identical)
        self._pack = {}
        keys = set(self._near_d) | set(self._near_c)
        for k in keys:
            dd = tuple((self.discs[i][0], self.discs[i][1],
                        self.discs[i][2])
                       for i in self._near_d.get(k, ()))
            cc = []
            for ci in self._near_c.get(k, ()):
                (ax, ay), (bx, by), r, _n = self.caps[ci]
                dx, dy = bx - ax, by - ay
                cc.append((ax, ay, dx, dy, dx * dx + dy * dy, r))
            self._pack[k] = (dd, tuple(cc))

    def near_discs(self, p):
        nd = getattr(self, '_near_d', None)
        if nd is not None:
            return nd.get((int(p[0] / self.cell),
                           int(p[1] / self.cell)), ())
        key = (int(p[0] / self.cell), int(p[1] / self.cell))
        out = []
        for dx_ in (-1, 0, 1):
            for dy_ in (-1, 0, 1):
                out.extend(self._grid.get((key[0] + dx_, key[1] + dy_), ()))
        return out

    def near_caps(self, p):
        nc = getattr(self, '_near_c', None)
        if nc is not None:
            return nc.get((int(p[0] / self.cell),
                           int(p[1] / self.cell)), ())
        cg = getattr(self, '_cgrid', None)
        if cg is None:
            return range(len(self.caps))
        key = (int(p[0] / self.cell), int(p[1] / self.cell))
        out = set()
        for dx_ in (-1, 0, 1):
            for dy_ in (-1, 0, 1):
                out.update(cg.get((key[0] + dx_, key[1] + dy_), ()))
        return out

    def point_violation(self, p, pad=0.0):
        """Deepest violated obstacle at point p -> (depth, push_dir) or
        None. `pad` inflates every obstacle radius (e.g. for a via body
        wider than the track the margins were built for).

        The built path runs on the per-cell geometry packs: identical
        candidates, identical order, identical arithmetic (math.hypot
        kept -- numpy measured 3x SLOWER per call at ~14 candidates),
        just no per-candidate indexing/unpacking/call frames, and the
        push direction computed once for the winner instead of per
        improvement. Bit-identical results, ~2x fewer frames."""
        pk = getattr(self, '_pack', None)
        if pk is not None:
            px, py = p
            cell = pk.get((int(px / self.cell), int(py / self.cell)))
            if cell is None:
                return None
            discs, caps = cell
            hyp = math.hypot
            best = 0.0
            wdisc = wcap = None
            for x, y, r in discs:
                d = hyp(px - x, py - y)
                depth = r + pad - d
                if depth > best:
                    best = depth
                    wdisc = (x, y, d)
            for ax, ay, dx, dy, L2, r in caps:
                if L2 < 1e-12:
                    tt = 0.0
                else:
                    tt = ((px - ax) * dx + (py - ay) * dy) / L2
                    if tt < 0.0:
                        tt = 0.0
                    elif tt > 1.0:
                        tt = 1.0
                d = hyp(px - (ax + tt * dx), py - (ay + tt * dy))
                depth = r + pad - d
                if depth > best:
                    best = depth
                    wdisc = None
                    wcap = (ax, ay, dx, dy, L2, tt, d)
            if wcap is not None:
                ax, ay, dx, dy, L2, tt, d = wcap
                cx, cy = ax + tt * dx, ay + tt * dy
                if d < 1e-9:
                    dirv = (-dy / math.sqrt(L2), dx / math.sqrt(L2)) \
                        if L2 > 1e-12 else (1.0, 0.0)
                else:
                    dirv = ((px - cx) / d, (py - cy) / d)
                return (best, dirv)
            if wdisc is not None:
                x, y, d = wdisc
                dirv = (1.0, 0.0) if d < 1e-9 else \
                    ((px - x) / d, (py - y) / d)
                return (best, dirv)
            return None
        worst = None
        for i in self.near_discs(p):
            x, y, r, _n = self.discs[i]
            r += pad
            d = math.hypot(p[0] - x, p[1] - y)
            if d < r:
                depth = r - d
                if worst is None or depth > worst[0]:
                    if d < 1e-9:
                        dirv = (1.0, 0.0)
                    else:
                        dirv = ((p[0] - x) / d, (p[1] - y) / d)
                    worst = (depth, dirv)
        for ci in self.near_caps(p):
            a, b, r, _n = self.caps[ci]
            r += pad
            d = seg_pt_dist(a, b, p)
            if d < r:
                depth = r - d
                if worst is None or depth > worst[0]:
                    # push perpendicular away from capsule axis
                    ax, ay = a
                    bx, by = b
                    dx, dy = bx - ax, by - ay
                    L2 = dx * dx + dy * dy
                    t = 0.0 if L2 < 1e-12 else max(
                        0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy)
                                 / L2))
                    cx, cy = ax + t * dx, ay + t * dy
                    dd = math.hypot(p[0] - cx, p[1] - cy)
                    if dd < 1e-9:
                        dirv = (-dy / math.sqrt(L2), dx / math.sqrt(L2)) \
                            if L2 > 1e-12 else (1.0, 0.0)
                    else:
                        dirv = ((p[0] - cx) / dd, (p[1] - cy) / dd)
                    worst = (depth, dirv)
        return worst

    def seg_clear(self, a, b):
        # disc candidates from the grid, sampled along the segment
        # (sample step 0.9 < cell 1.0, so with the 3x3 neighbourhood
        # every cell the segment touches is covered); caps via a bbox
        # prefilter. Exact checks on the candidates only.
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        nsteps = max(1, int(L / 0.9))
        cand = set()
        for k in range(nsteps + 1):
            t = k / nsteps
            cand.update(self.near_discs((a[0] + t * (b[0] - a[0]),
                                         a[1] + t * (b[1] - a[1]))))
        for i in cand:
            x, y, r, _n = self.discs[i]
            if seg_pt_dist(a, b, (x, y)) < r:
                return False
        xlo, xhi = min(a[0], b[0]), max(a[0], b[0])
        ylo, yhi = min(a[1], b[1]), max(a[1], b[1])
        ccand = set()
        for k in range(nsteps + 1):
            t = k / nsteps
            ccand.update(self.near_caps((a[0] + t * (b[0] - a[0]),
                                         a[1] + t * (b[1] - a[1]))))
        for ci in ccand:
            c, e, r, _n = self.caps[ci]
            if min(c[0], e[0]) - r > xhi or max(c[0], e[0]) + r < xlo \
                    or min(c[1], e[1]) - r > yhi \
                    or max(c[1], e[1]) + r < ylo:
                continue
            if seg_seg_dist(a, b, c, e) < r:
                return False
        return True

    def hugs(self, pts, slack=0.05):
        names = set()
        for p in pts:
            for i in self.near_discs(p):
                x, y, r, n = self.discs[i]
                if math.hypot(p[0] - x, p[1] - y) < r + slack:
                    names.add(n)
            for ci in self.near_caps(p):
                a, b, r, n = self.caps[ci]
                if seg_pt_dist(a, b, p) < r + slack:
                    names.add(n)
        return names


def densify(pts, step=STEP):
    out = [pts[0]]
    for q in pts[1:]:
        p = out[-1]
        n = max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / step))
        for k in range(1, n + 1):
            out.append((p[0] + (q[0] - p[0]) * k / n,
                        p[1] + (q[1] - p[1]) * k / n))
    return out


def shortcut(pts, obs):
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not obs.seg_clear(pts[i], pts[j]):
            j = (i + j) // 2 if j - i > 8 else j - 1
        # linear fallback ensures progress
        while j > i + 1 and not obs.seg_clear(pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def relax(src, dst, obs, rounds=400):
    pts = densify([src, dst])
    ends = (src, dst)
    for it in range(rounds):
        moved = 0.0
        for i in range(1, len(pts) - 1):
            p = pts[i]
            if d2(p, ends[0]) < FREEZE ** 2 or d2(p, ends[1]) < FREEZE ** 2:
                continue
            q = (0.5 * p[0] + 0.25 * pts[i - 1][0] + 0.25 * pts[i + 1][0],
                 0.5 * p[1] + 0.25 * pts[i - 1][1] + 0.25 * pts[i + 1][1])
            for _k in range(6):
                v = obs.point_violation(q)
                if v is None:
                    break
                depth, (ux, uy) = v
                q = (q[0] + ux * (depth + 0.01), q[1] + uy * (depth + 0.01))
            moved += math.hypot(q[0] - p[0], q[1] - p[1])
            pts[i] = q
        if it % 25 == 24:
            pts = densify(shortcut(pts, obs))
        if moved < 1e-4 * len(pts):
            break
    return densify(shortcut(pts, obs)), it + 1


def polyline_len(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(pts, pts[1:]))


def crossings(sa, sb, end_excl=0.5, dedupe=0.4):
    hits = []
    for i in range(len(sa) - 1):
        for j in range(len(sb) - 1):
            p = seg_x(sa[i], sa[i + 1], sb[j], sb[j + 1])
            if p is None:
                continue
            if min(d2(p, sa[0]), d2(p, sa[-1]), d2(p, sb[0]),
                   d2(p, sb[-1])) < end_excl ** 2:
                continue
            hits.append(p)
    out = []
    for h in hits:
        if all(d2(h, o) > dedupe ** 2 for o in out):
            out.append(h)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', default=os.path.join(HERE,
                    'fb_t2q_base.kicad_pcb'))
    ap.add_argument('--nets', default='SDQ15,SDQ14,SDQ13,SDQ11')
    ap.add_argument('--out', default=os.path.join(HERE, 'topo_k4'))
    a = ap.parse_args()
    names = [n.strip() for n in a.nets.split(',') if n.strip()]

    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    kids = {byname[nm][0] for nm in names}

    # endpoints: source tooth free end (stub census) -> target pad
    ends = {}
    tgt_comp = {}
    for nm in names:
        nid, net = byname[nm]
        segs = [s for s in pcb.segments if s.net_id == nid]
        cnt = Counter()
        for s in segs:
            cnt[(round(s.start_x, 3), round(s.start_y, 3))] += 1
            cnt[(round(s.end_x, 3), round(s.end_y, 3))] += 1
        padpts = {(round(p.global_x, 3), round(p.global_y, 3)): p
                  for p in net.pads}
        free = [pt for pt, c in cnt.items() if c == 1 and pt not in padpts]
        if len(free) != 1:
            print(f'{nm}: expected 1 free end, got {free} -- skipping')
            continue
        src = free[0]
        tgt = max(net.pads, key=lambda p: d2((p.global_x, p.global_y), src))
        ends[nm] = (src, (tgt.global_x, tgt.global_y))
        tgt_comp[nm] = tgt.component_ref

    # target field bbox (union over requested nets' target components)
    fields = {}
    for comp in set(tgt_comp.values()):
        fp = pcb.footprints[comp]
        xs = [p.global_x for p in fp.pads]
        ys = [p.global_y for p in fp.pads]
        fields[comp] = (min(xs) - 0.4, min(ys) - 0.4,
                        max(xs) + 0.4, max(ys) + 0.4)

    def in_field(x, y):
        return any(x0 <= x <= x1 and y0 <= y <= y1
                   for (x0, y0, x1, y1) in fields.values())

    # obstacles are per-net (own copper excluded)
    strings = {}
    iters = {}
    for nm in names:
        if nm not in ends:
            continue
        nid, _ = byname[nm]
        obs = Obstacles()
        for ref, fp in pcb.footprints.items():
            for p in fp.pads:
                if p.net_id == nid:
                    continue
                if p.pad_type == 'np_thru_hole' and p.drill:
                    r0 = p.drill / 2
                elif p.shape in ('circle', 'oval'):
                    r0 = max(p.size_x, p.size_y) / 2
                else:
                    r0 = math.hypot(p.size_x, p.size_y) / 2
                m = MARGIN_IN if in_field(p.global_x, p.global_y) \
                    else MARGIN_OUT
                obs.add_disc(p.global_x, p.global_y, r0 + m,
                             f'{ref}.{p.pad_number}')
        for s in pcb.segments:
            if s.net_id == nid or s.net_id in kids:
                continue  # own copper; K-net stubs handled at spread time
            if s.layer != 'F.Cu':
                continue  # layer-conditional: nominal-F strings only
            m = MARGIN_IN if in_field(s.start_x, s.start_y) else MARGIN_OUT
            obs.add_cap((s.start_x, s.start_y), (s.end_x, s.end_y),
                        s.width / 2 + m, f'seg:{s.net_id}@{s.layer}')
        for v in pcb.vias:
            if v.net_id == nid:
                continue
            m = MARGIN_IN if in_field(v.x, v.y) else MARGIN_OUT
            obs.add_disc(v.x, v.y, v.size / 2 + m, f'via:{v.net_id}')
        obs.build()
        src, dst = ends[nm]
        pts, its = relax(src, dst, obs)
        strings[nm] = pts
        iters[nm] = its
        hug = sorted(obs.hugs(pts))
        print(f'{nm}: len={polyline_len(pts):.2f}mm verts={len(pts)} '
              f'iters={its} hugs={hug}')

    # crossing matrices: straight chords vs taut strings
    order = [nm for nm in names if nm in strings]
    for label, getter in (('straight', lambda nm: [ends[nm][0], ends[nm][1]]),
                          ('taut', lambda nm: strings[nm])):
        tot = 0
        pairs = []
        for i in range(len(order)):
            for j in range(i + 1, len(order)):
                x = crossings(getter(order[i]), getter(order[j]))
                if x:
                    pairs.append((order[i], order[j], len(x),
                                  [(round(p[0], 2), round(p[1], 2))
                                   for p in x]))
                tot += len(x)
        print(f'\n{label} crossings: {tot}')
        for (na, nb, n, ps) in pairs:
            print(f'  {na} x {nb}: {n} at {ps}')

    lens = [polyline_len(strings[nm]) for nm in order]
    mean = sum(lens) / len(lens)
    sd = math.sqrt(sum((v - mean) ** 2 for v in lens) / len(lens))
    print(f'\nlengths: avg={mean:.2f} max={max(lens):.2f} '
          f'std={sd:.2f} ({", ".join(f"{nm}={polyline_len(strings[nm]):.2f}" for nm in order)})')

    # outputs: strings JSON + Eco2 overlay board
    with open(a.out + '_strings.json', 'w') as f:
        json.dump({nm: [[round(x, 4), round(y, 4)] for (x, y) in pts]
                   for nm, pts in strings.items()}, f)
    txt = open(a.board, encoding='utf-8').read()
    lines = []
    for nm, pts in strings.items():
        for p, q in zip(pts, pts[1:]):
            lines.append(
                f'  (gr_line (start {p[0]:.4f} {p[1]:.4f}) '
                f'(end {q[0]:.4f} {q[1]:.4f}) '
                f'(stroke (width 0.05) (type solid)) '
                f'(layer "Eco2.User"))\n')
    k = txt.rstrip().rfind(')')
    out_board = a.out + '.kicad_pcb'
    with open(out_board, 'w') as f:
        f.write(txt[:k] + ''.join(lines) + txt[k:])
    pro = os.path.splitext(a.board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, a.out + '.kicad_pro')
    print(f'wrote {out_board} (+{len(lines)} eco lines), '
          f'{a.out}_strings.json')


if __name__ == '__main__':
    main()
