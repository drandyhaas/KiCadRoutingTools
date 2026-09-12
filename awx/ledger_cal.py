#!/usr/bin/env python3
"""#622 calibrated floor evaluator (STEP 0 -> the trial judge).

Per K net on a routed board: the simple path tooth-ball ->
berth-ball through its real copper (T-joints, distance-merged nodes,
via-barrel-mediated adjacency -- each rule paid for by a control),
its REAL crossings with every other net's path (ordered, with the
partner's actual layer there), and the DP floor: the minimal layer
changes for a path holding its end layers and sitting opposite each
partner at each crossing, everyone else fixed.

CALIBRATED (0902, six boards): floors <= actuals everywhere, slack
uniformly 2-4 on human AND ours, true order reproduced at every rung
(incl. our K28 win). The board-vs-board gap is the FLOOR gap --
+8 at K35, +16 at K41 against the human -- i.e. the page/wrap
assignment, priced.

Judge.swap_floor(candidate, n) re-evaluates INCREMENTALLY (only n's
path and the partners it touches change -- the frozen-world screen
boards differ from the base by one net's copper), so the improve
loop's screens can judge a candidate by the quantity that matters:
does the FLOOR drop, not just this net's vias.

usage: ledger_cal.py BOARD K [--verbose]
"""
import argparse
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402


class Path:
    """One net's tooth->berth simple path: points, layer per edge,
    cumulative arclength, layer-change count."""

    def __init__(self, pcb, sfp, dfp, nid):
        segs = [s for s in pcb.segments if s.net_id == nid]
        canon = []

        def node(x, y):
            for i2, (cx, cy) in enumerate(canon):
                if abs(cx - x) < 0.02 and abs(cy - y) < 0.02:
                    return i2
            canon.append((x, y))
            return len(canon) - 1
        ends = [(node(s.start_x, s.start_y), node(s.end_x, s.end_y))
                for s in segs]
        adj = {}
        for si, (na, nb) in enumerate(ends):
            a2, b2 = canon[na], canon[nb]
            adj.setdefault(a2, []).append((b2, si))
            adj.setdefault(b2, []).append((a2, si))
        # T-joints: an endpoint on a same-layer segment's INTERIOR
        for pt in list(adj.keys()):
            for si, s in enumerate(segs):
                a2 = canon[ends[si][0]]
                b2 = canon[ends[si][1]]
                if pt == a2 or pt == b2:
                    continue
                dx = s.end_x - s.start_x
                dy = s.end_y - s.start_y
                L2 = dx * dx + dy * dy
                if L2 < 1e-12:
                    continue
                t = ((pt[0] - s.start_x) * dx
                     + (pt[1] - s.start_y) * dy) / L2
                if not (0.02 < t < 0.98):
                    continue
                d = math.hypot(pt[0] - (s.start_x + t * dx),
                               pt[1] - (s.start_y + t * dy))
                if d < 0.02:
                    adj.setdefault(pt, []).append((a2, si))
                    adj.setdefault(pt, []).append((b2, si))
                    adj.setdefault(a2, []).append((pt, si))
                    adj.setdefault(b2, []).append((pt, si))
        # via-barrel-mediated adjacency (si = -1: zero-length hop)
        for v in pcb.vias:
            if v.net_id != nid:
                continue
            on = [p2 for p2 in list(adj.keys())
                  if math.hypot(p2[0] - v.x, p2[1] - v.y)
                  <= v.size / 2 + 0.02]
            for i2 in range(len(on)):
                for j2 in range(i2 + 1, len(on)):
                    adj[on[i2]].append((on[j2], -1))
                    adj[on[j2]].append((on[i2], -1))
        sball = next((p for p in sfp.pads if p.net_id == nid), None)
        dball = next((p for p in dfp.pads if p.net_id == nid), None)
        self.ok = False
        if not sball or not dball or not segs:
            return

        def pad_layer(p):
            for L in p.layers:
                if L.endswith('.Cu') and '*' not in L:
                    return L
            return 'F.Cu'
        # the TRUE terminals are the PAD layers: a via-in-pad B ride
        # has no F copper at all, so first-run terminals counted its
        # 2 real vias as 0 changes (measured: SA13, and the
        # cross-board floor comparison flattered via-in-pad-heavy
        # boards)
        self.start_pad_lay = pad_layer(sball)
        self.end_pad_lay = pad_layer(dball)

        def nearest(x, y):
            return min(adj, key=lambda n2: (n2[0] - x) ** 2
                       + (n2[1] - y) ** 2)
        src = nearest(sball.global_x, sball.global_y)
        dst = nearest(dball.global_x, dball.global_y)
        prev = {src: None}
        q = [src]
        while q:
            cur = q.pop(0)
            if cur == dst:
                break
            for (nxt, si) in adj.get(cur, []):
                if nxt not in prev:
                    prev[nxt] = (cur, si)
                    q.append(nxt)
        if dst not in prev:
            return
        chain = []
        cur = dst
        while prev[cur] is not None:
            pcur, si = prev[cur]
            chain.append((pcur, cur, si))
            cur = pcur
        chain.reverse()
        if not chain:
            return
        self.pts = [chain[0][0]] + [c[1] for c in chain]
        self.lay = [segs[c[2]].layer if c[2] >= 0 else None
                    for c in chain]
        for i2 in range(len(self.lay) - 1, -1, -1):
            if self.lay[i2] is None:
                self.lay[i2] = (self.lay[i2 + 1]
                                if i2 + 1 < len(self.lay)
                                and self.lay[i2 + 1] is not None
                                else 'F.Cu')
        for i2 in range(len(self.lay)):
            if self.lay[i2] is None:
                self.lay[i2] = self.lay[i2 - 1] if i2 else 'F.Cu'
        self.cum = [0.0]
        for (p1, p2, _si) in chain:
            self.cum.append(self.cum[-1] + math.hypot(
                p2[0] - p1[0], p2[1] - p1[1]))
        self.changes = sum(1 for i2 in range(1, len(self.lay))
                           if self.lay[i2] != self.lay[i2 - 1])
        self.changes += (1 if self.lay[0] != self.start_pad_lay
                         else 0)
        self.changes += (1 if self.lay[-1] != self.end_pad_lay
                         else 0)
        self.ok = True

    def layer_at(self, t):
        for i in range(len(self.lay)):
            if self.cum[i] <= t <= self.cum[i + 1] + 1e-9:
                return self.lay[i]
        return self.lay[-1]


def _seg_x(p1, p2, p3, p4):
    r = (p2[0] - p1[0], p2[1] - p1[1])
    s2 = (p4[0] - p3[0], p4[1] - p3[1])
    den = r[0] * s2[1] - r[1] * s2[0]
    if abs(den) < 1e-12:
        return None
    t = ((p3[0] - p1[0]) * s2[1] - (p3[1] - p1[1]) * s2[0]) / den
    u = ((p3[0] - p1[0]) * r[1] - (p3[1] - p1[1]) * r[0]) / den
    if 1e-6 < t < 1 - 1e-6 and 1e-6 < u < 1 - 1e-6:
        return t, u
    return None


def _cross_pair(A, B):
    """[(ta, tb)] intersections of two paths."""
    out = []
    for ia in range(len(A.pts) - 1):
        for ib in range(len(B.pts) - 1):
            r = _seg_x(A.pts[ia], A.pts[ia + 1],
                       B.pts[ib], B.pts[ib + 1])
            if r is None:
                continue
            t, u = r
            out.append((A.cum[ia] + t * (A.cum[ia + 1] - A.cum[ia]),
                        B.cum[ib] + u * (B.cum[ib + 1] - B.cum[ib])))
    return out


def _dp(path, seq):
    """Min layer changes given the ordered requirement sequence
    [(t, partner_layer)] -- the path must be OPPOSITE each. The
    terminals are the PAD layers (via-in-pad terminals are real
    changes)."""
    cur = path.start_pad_lay
    ch = 0
    for (_t, pl) in sorted(seq):
        want = 'B.Cu' if pl == 'F.Cu' else 'F.Cu'
        if want != cur:
            ch += 1
            cur = want
    if path.end_pad_lay != cur:
        ch += 1
    return ch


class Judge:
    """Calibrated floor evaluator with incremental candidate
    re-scoring (swap_floor)."""

    def __init__(self, board, nets, src='U1', dst='DU1'):
        self.nets = list(nets)
        self.src, self.dst = src, dst
        pcb = parse_kicad_pcb(board)
        self.short = {i: n.name.rsplit('/', 1)[-1]
                      for i, n in pcb.nets.items()}
        self.nid = {v: k for k, v in self.short.items()}
        sfp, dfp = pcb.footprints[src], pcb.footprints[dst]
        self.paths = {}
        for nid, n in pcb.nets.items():
            nm = self.short.get(nid)
            if nm in nets:
                p = Path(pcb, sfp, dfp, nid)
                if p.ok:
                    self.paths[nm] = p
        self.seqs = {m: [] for m in self.paths}
        names = sorted(self.paths)
        for i, m in enumerate(names):
            for o in names[i + 1:]:
                for (ta, tb) in _cross_pair(self.paths[m],
                                            self.paths[o]):
                    self.seqs[m].append(
                        (ta, o, self.paths[o].layer_at(tb)))
                    self.seqs[o].append(
                        (tb, m, self.paths[m].layer_at(ta)))
        self.per = {}
        for m in names:
            self.per[m] = _dp(self.paths[m],
                              [(t, pl) for (t, _o, pl)
                               in self.seqs[m]])
        self.floor_total = sum(self.per.values())
        self.act_total = sum(p.changes for p in self.paths.values())

    def swap_floor(self, cand_board, n):
        """Floor of the base board with net n's copper replaced by
        the candidate's -- incremental: only n's path and the
        partners it touches are re-evaluated. Returns None when the
        candidate has no path for n."""
        cpcb = parse_kicad_pcb(cand_board)
        nid = next((i for i, net in cpcb.nets.items()
                    if net.name.rsplit('/', 1)[-1] == n), None)
        if nid is None:
            return None
        np_ = Path(cpcb, cpcb.footprints[self.src],
                   cpcb.footprints[self.dst], nid)
        if not np_.ok:
            return None
        total = self.floor_total
        nseq = []
        touched = {}
        for o, P in self.paths.items():
            if o == n:
                continue
            for (ta, tb) in _cross_pair(np_, P):
                nseq.append((ta, P.layer_at(tb)))
                touched.setdefault(o, []).append(
                    (tb, np_.layer_at(ta)))
        for o in set(list(touched) +
                     [o for (_t, o, _pl) in self.seqs.get(n, [])]):
            old = self.per[o]
            keep = [(t, pl) for (t, o2, pl) in self.seqs[o]
                    if o2 != n]
            new = _dp(self.paths[o], keep + touched.get(o, []))
            total += new - old
        total += _dp(np_, nseq) - self.per.get(n, 0)
        return total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k')
    ap.add_argument('--src', default='U1')
    ap.add_argument('--dst', default='DU1')
    ap.add_argument('--verbose', action='store_true')
    a = ap.parse_args()
    nets = subprocess.run(
        [sys.executable, os.path.join(HERE, 'coherent_nets.py'), a.k],
        capture_output=True, text=True).stdout.strip().split(',')
    J = Judge(a.board, nets, a.src, a.dst)
    missing = [m for m in nets if m not in J.paths]
    for m in missing:
        print(f'  (no path for {m})')
    ncross = sum(len(v) for v in J.seqs.values()) // 2
    print(f'{os.path.basename(a.board)} K={a.k}: '
          f'{len(J.paths)} paths, {ncross} real crossings')
    print(f'  actual changes on paths: {J.act_total}')
    print(f'  DP floor (others fixed): {J.floor_total}')
    print(f'  slack (realization headroom): '
          f'{J.act_total - J.floor_total}')
    bad = 0
    for m in sorted(J.paths):
        for (ta, o, pl) in J.seqs[m]:
            if m < o and J.paths[m].layer_at(ta) == pl:
                bad += 1
    if bad:
        print(f'  WARNING: {bad} same-layer crossing reading(s)')
    if a.verbose:
        rows = sorted(((J.paths[m].changes - J.per[m], m)
                       for m in J.paths), reverse=True)
        for d, m in rows[:12]:
            print(f'    {m:7s} act {J.paths[m].changes} '
                  f'dp {J.per[m]} ({len(J.seqs[m])} crossings)')


if __name__ == '__main__':
    main()
