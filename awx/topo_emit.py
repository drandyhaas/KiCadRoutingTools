#!/usr/bin/env python3
"""Braid-schedule emitter (v2) for the #622 topo-string experiment.

Pipeline: endpoints from the stub census; target order = BALL y; divers =
complement of the LIS of the launch->ball permutation; adjacent swaps
scheduled by bubbling divers one at a time in ascending target rank (each
diver's swaps form an exclusive x-band, so diver-diver swaps are
automatically exactly-one-on-B); trunk drawn as morphing lane slots.

Field entry, two modes:
 - F street: fanout-style run along the empty inter-row street, half-pitch
   jog to the ball. F entries must be strictly increasing in target order
   (tails cannot cross).
 - B dogbone: B.Cu under the field is EMPTY (all pads F-only SMD), so a
   net that cannot get a street enters on B and surfaces at a via in the
   diagonal cell next to its ball. For a diver this costs no extra vias
   (its surface via moves to the dogbone site; east-extending a window is
   safe: all swaps involving a diver live in its own or earlier bands).
   A promoted non-diver pays +2 vias. B entries keep their own monotone
   chain; in the trunk they ride a PHANTOM monotone lane and fan to the
   true entry on B east of the splice.

Emits copper (segments + vias) into a board copy for check_connected /
check_drc grading, plus an Eco2 overlay of the same geometry.
"""
import argparse
import math
import re
import os
import shutil
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_strings as ts  # noqa: E402

TRACK = 0.127
CLEAR = 0.105            # 0.1 spec + 5um so hugs don't sit exactly at 0.1
VIA_SIZE = 0.25
VIA_DRILL = 0.15


def build_obstacles(pcb, nid, kids, layer):
    obs = ts.Obstacles()
    m = CLEAR + TRACK / 2
    for ref, fp in pcb.footprints.items():
        for p in fp.pads:
            if p.net_id == nid:
                continue
            on_layer = any(L == layer or '*' in L for L in p.layers)
            if p.drill and p.drill > 0:
                on_layer = True
            if not on_layer:
                continue
            if p.pad_type == 'np_thru_hole' and p.drill:
                r0 = p.drill / 2
            elif p.shape in ('circle', 'oval'):
                r0 = max(p.size_x, p.size_y) / 2
            else:
                r0 = math.hypot(p.size_x, p.size_y) / 2
            obs.add_disc(p.global_x, p.global_y, r0 + m,
                         f'{ref}.{p.pad_number}')
    for s in pcb.segments:
        if s.net_id == nid or s.net_id in kids or s.layer != layer:
            continue
        obs.add_cap((s.start_x, s.start_y), (s.end_x, s.end_y),
                    s.width / 2 + m, f'seg:{s.net_id}')
    for v in pcb.vias:
        if v.net_id == nid:
            continue
        obs.add_disc(v.x, v.y, v.size / 2 + m, f'via:{v.net_id}')
    obs.build()
    return obs


def endpoints(pcb, names, byname):
    ends = {}
    for nm in names:
        nid, net = byname[nm]
        segs = [s for s in pcb.segments if s.net_id == nid]
        cnt = Counter()
        for s in segs:
            cnt[(round(s.start_x, 3), round(s.start_y, 3))] += 1
            cnt[(round(s.end_x, 3), round(s.end_y, 3))] += 1
        padpts = {(round(p.global_x, 3), round(p.global_y, 3))
                  for p in net.pads}
        free = [pt for pt, c in cnt.items() if c == 1 and pt not in padpts]
        assert len(free) == 1, (nm, free)
        src = free[0]
        tgt = max(net.pads, key=lambda p: ts.d2((p.global_x, p.global_y),
                                                src))
        ends[nm] = (src, (tgt.global_x, tgt.global_y), tgt.component_ref)
    return ends


def lis_keep(ranks):
    n = len(ranks)
    best = [1] * n
    prev = [-1] * n
    for i in range(n):
        for j in range(i):
            if ranks[j] < ranks[i] and best[j] + 1 > best[i]:
                best[i] = best[j] + 1
                prev[i] = j
    i = max(range(n), key=lambda k: best[k])
    keep = set()
    while i >= 0:
        keep.add(i)
        i = prev[i]
    return keep


def rdp(pts, eps=0.02):
    """Ramer-Douglas-Peucker polyline simplification."""
    if len(pts) < 3:
        return list(pts)
    a, b = pts[0], pts[-1]
    dmax, idx = 0.0, 0
    for i in range(1, len(pts) - 1):
        d = ts.seg_pt_dist(a, b, pts[i])
        if d > dmax:
            dmax, idx = d, i
    if dmax <= eps:
        return [a, b]
    left = rdp(pts[:idx + 1], eps)
    return left[:-1] + rdp(pts[idx:], eps)


def octify_seg(p, q, eps=0.05):
    """Decompose one arbitrary-angle segment into octilinear pieces that
    stay within ~eps of the original: K interleaved blocks, each
    half-axis / 45 / half-axis (centered look). Steep segments become
    45/vertical staircases, shallow ones long axis runs with gentle 45
    nudges."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    adx, ady = abs(dx), abs(dy)
    if adx < 1e-9 or ady < 1e-9 or abs(adx - ady) < 1e-9:
        return [p, q]
    m = min(adx, ady)               # total 45 travel (per axis)
    r = abs(adx - ady)              # dominant-axis remainder
    K = max(1, math.ceil(r * m / ((adx + ady) * 2 * eps)))
    sx = 1 if dx > 0 else -1
    sy = 1 if dy > 0 else -1
    dom_x = adx >= ady
    pts = [p]
    x, y = p
    for _k in range(K):
        for frac in (0.5, None, 0.5):
            if frac is None:
                x += sx * m / K
                y += sy * m / K
            elif dom_x:
                x += sx * r / K * frac
            else:
                y += sy * r / K * frac
            pts.append((x, y))
    pts[-1] = q
    return pts


def merge_collinear(pts):
    out = [pts[0]]
    for p in pts[1:]:
        if abs(p[0] - out[-1][0]) + abs(p[1] - out[-1][1]) < 1e-9:
            continue
        if len(out) >= 2:
            ax, ay = out[-2]
            bx, by = out[-1]
            d1 = (bx - ax, by - ay)
            d2 = (p[0] - bx, p[1] - by)
            if abs(d1[0] * d2[1] - d1[1] * d2[0]) < 1e-9 and \
                    d1[0] * d2[0] + d1[1] * d2[1] > 0:
                out[-1] = p
                continue
        out.append(p)
    return out


def octify_segs(segs, eps=0.05, fine=None):
    """Octilinearize a net's layered segment chain: per constant-layer
    run, RDP-simplify (the morphing-lane points are collinear between
    swaps and merge away), then octify each remaining segment. `fine`
    = (windows, fine_eps): segments overlapping a window x-range use
    the tighter tube (verifier-driven local refinement)."""
    out = []
    i = 0
    while i < len(segs):
        layer = segs[i][2]
        pts = [segs[i][0], segs[i][1]]
        j = i
        while j + 1 < len(segs) and segs[j + 1][2] == layer and \
                segs[j + 1][0] == segs[j][1]:
            j += 1
            pts.append(segs[j][1])
        run = rdp(pts, 0.02)
        opts = [run[0]]
        for p, q in zip(run, run[1:]):
            e = eps
            if fine:
                wlist, fe = fine
                xlo, xhi = min(p[0], q[0]), max(p[0], q[0])
                if any(xlo <= b and a <= xhi for (a, b) in wlist):
                    e = fe
            opts.extend(octify_seg(p, q, e)[1:])
        opts = merge_collinear(opts)
        out.extend((p, q, layer) for p, q in zip(opts, opts[1:]))
        i = j + 1
    return out


def strip_net_segments(txt, net_ids):
    """Remove every (segment ...) block whose (net N) is in net_ids,
    paren-balanced (handles both KiCad multi-line and our one-line
    forms)."""
    out = []
    i = 0
    while True:
        j = txt.find('(segment', i)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            c = txt[k]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        m = re.search(r'\(net (\d+)\)', txt[j:k + 1])
        if m and int(m.group(1)) in net_ids:
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', default=os.path.join(HERE,
                    'fb_t2q_base.kicad_pcb'))
    ap.add_argument('--nets', default='SDQ15,SDQ14,SDQ13,SDQ11')
    ap.add_argument('--out', default=os.path.join(HERE, 'topo_k4_emit'))
    ap.add_argument('--pitch', type=float, default=0.8)
    ap.add_argument('--dump-segs', default='')
    ap.add_argument('--no-smooth', action='store_true',
                    help='skip the repo #536 octolinear smoothing pass')
    ap.add_argument('--moves', default='',
                    help='homotopy entry moves, e.g. SDQ0=S,SDQ11=N')
    ap.add_argument('--no-octi', action='store_true',
                    help='emit the raw arbitrary-angle braid geometry')
    a = ap.parse_args()
    names = [n.strip() for n in a.nets.split(',') if n.strip()]

    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    kids = {byname[nm][0] for nm in names}
    ends = endpoints(pcb, names, byname)

    comps = {ends[nm][2] for nm in names}
    fx0 = min(p.global_x for c in comps for p in pcb.footprints[c].pads)
    x1 = fx0 - a.pitch * 0.8
    x0 = max(ends[nm][0][0] for nm in names) + 0.3

    launch = sorted(names, key=lambda nm: ends[nm][0][1])
    print(f'launch order: {launch}')

    obsF = {nm: build_obstacles(pcb, byname[nm][0], kids, 'F.Cu')
            for nm in names}
    half = a.pitch / 2
    seg_min = TRACK + CLEAR                   # placed-vs-placed center min
    via_seg_min = VIA_SIZE / 2 + TRACK / 2 + CLEAR

    # entry assignment (ball-y order). F candidates: DIRECT (column-A
    # ball: enter at its own row y, no street consumed), north street,
    # south street. Leftovers get a B dogbone site. Candidates must be
    # static-clear and clear of already-placed tails (layer-aware).
    entry = {}          # nm -> ('F', entry_y) | ('B', (sx, sy))
    placed_f, placed_vias = [], []

    def f_ok(nm, segs):
        for (p, q) in segs:
            if p == q:
                continue
            if not obsF[nm].seg_clear(p, q):
                return False
            for (c, e) in placed_f:
                if ts.seg_seg_dist(p, q, c, e) < seg_min:
                    return False
            for v in placed_vias:
                if ts.seg_pt_dist(p, q, v) < via_seg_min:
                    return False
        return True

    moves = {}
    for tok in a.moves.split(','):
        if '=' in tok:
            k, v = tok.split('=')
            moves[k.strip()] = v.strip().upper()
    comps_pads = [p for c in comps for p in pcb.footprints[c].pads]
    rows_all = sorted({round(p.global_y, 3) for p in comps_pads})

    ball_order = sorted(names, key=lambda nm: (ends[nm][1][1],
                                               ends[nm][1][0]))
    for nm in ball_order:
        if nm in moves:
            continue
        bx, by = ends[nm][1][0], ends[nm][1][1]
        cands = []
        if bx <= fx0 + 0.01:
            cands.append(('direct', by, [((x1, by), (bx, by))]))
        for sy in (by - half, by + half):
            cands.append(('street', sy, [((x1, sy), (bx, sy)),
                                         ((bx, sy), (bx, by))]))
        for kind, ey, segs in cands:
            if any(abs(ey - e[1]) < 0.3 for e in entry.values()
                   if e[0] == 'F'):
                continue
            if f_ok(nm, segs):
                entry[nm] = ('F', ey)
                placed_f.extend(s for s in segs if s[0] != s[1])
                break

    # flank entries (#622 take-3 task 2): trunk lane just outside the W
    # block (clears the corridor cap band), vertical drop at the splice
    # column to the outer N/S run along the field edge, then the
    # vertical inter-column street west of the ball, half-pitch jog.
    f_ys = [e[1] for e in entry.values() if e[0] == 'F']
    for flank in ('N', 'S'):
        movers = sorted((nm for nm, f in moves.items() if f == flank),
                        key=lambda nm: ends[nm][1][0])
        for i, nm in enumerate(movers):
            bx, by = ends[nm][1][0], ends[nm][1][1]
            sx = bx - half
            if flank == 'N':
                run_y = rows_all[0] - 0.4 - i * 0.3
                ly = (min(f_ys) if f_ys else rows_all[0]) - 0.35 - i * 0.3
                dx_ = x1   # single-file drop; multi-S staggering TBD
            else:
                run_y = rows_all[-1] + 0.4 + i * 0.3
                ly = (max(f_ys) if f_ys else rows_all[-1]) + 0.35 + i * 0.3
                dx_ = x1   # single-file drop; multi-S staggering TBD
            entry[nm] = (flank, (sx, run_y, ly, dx_))
            placed_f.append(((dx_, ly), (dx_, run_y)))
            placed_f.append(((dx_, run_y), (sx, run_y)))
            placed_f.append(((sx, run_y), (sx, by)))
            placed_f.append(((sx, by), (bx, by)))

    for nm in ball_order:
        if nm in entry or nm in moves:
            continue
        bx, by = ends[nm][1][0], ends[nm][1][1]
        sx = bx - half
        chosen = None
        for sy in (by - half, by + half):
            ok = all(ts.seg_pt_dist(p, q, (sx, sy)) > via_seg_min
                     for (p, q) in placed_f)
            ok = ok and all(math.hypot(sx - v[0], sy - v[1]) >
                            VIA_SIZE + CLEAR for v in placed_vias)
            if ok and f_ok(nm, [((sx, sy), (bx, by))]):
                chosen = sy
                break
        assert chosen is not None, f'no street and no dogbone for {nm}'
        entry[nm] = ('B', (sx, chosen))
        placed_f.append(((sx, chosen), (bx, by)))
        placed_vias.append((sx, chosen))

    # lane ys: F nets pin their entry; B nets slot strictly between the
    # neighbouring F entries, ordered by site y
    f_sorted = sorted([(e[1], nm) for nm, e in entry.items()
                       if e[0] == 'F']
                      + [(e[1][2], nm) for nm, e in entry.items()
                         if e[0] in ('N', 'S')])
    b_nets = sorted(((e[1][1], nm) for nm, e in entry.items()
                     if e[0] == 'B'))
    lane_y = {nm: ey for ey, nm in f_sorted}
    for sy, nm in b_nets:
        fy = [v[0] for v in f_sorted]
        lo = max([y for y in fy if y <= sy], default=sy - 0.4)
        hi = min([y for y in fy if y > sy], default=sy + 0.4)
        cand = (lo + hi) / 2
        while any(abs(cand - v) < 0.25 for v in lane_y.values()):
            cand += 0.27
        lane_y[nm] = cand

    target = sorted(names, key=lambda nm: lane_y[nm])
    trank = {nm: i for i, nm in enumerate(target)}
    ranks = [trank[nm] for nm in launch]
    keep = lis_keep(ranks)
    divers = {launch[i] for i in range(len(launch)) if i not in keep}
    promoted = [nm for nm in names
                if entry[nm][0] == 'B' and nm not in divers]
    divers |= set(promoted)
    print(f'entry order (lane y): {target}  ranks: {ranks}')
    print(f'divers ({len(divers)}): {sorted(divers, key=lambda n: trank[n])}')
    for nm in target:
        mode, v = entry[nm]
        print(f'{nm}: {mode}-entry {v} lane_y={lane_y[nm]:.3f}'
              + (' (PROMOTED +2 vias)' if nm in promoted else ''))

    # swap schedule: each diver moves ONCE to the slot just after the last
    # smaller-rank element. West-movers (final rank earlier than launch
    # position) go first in ascending rank, then east-movers in descending
    # rank -- this ordering makes every positional pass a genuine
    # inversion, so windows stay serial-band-consistent.
    west = [d for d in divers if trank[d] < launch.index(d)]
    east = [d for d in divers if trank[d] >= launch.index(d)]
    order_of_moves = sorted(west, key=lambda nm: trank[nm]) + \
        sorted(east, key=lambda nm: -trank[nm])
    seq = list(launch)
    swaps = []
    for d in order_of_moves:
        i = seq.index(d)
        rest = seq[:i] + seq[i + 1:]
        want = 0
        for j, e in enumerate(rest):
            if trank[e] < trank[d]:
                want = j + 1
        passed = rest[want:i] if want <= i else rest[i:want]
        for e in (reversed(passed) if want <= i else passed):
            swaps.append((d, e))
        seq = rest[:want] + [d] + rest[want:]
    assert seq == target, (seq, target)
    seen_pairs = set()
    for (d, e) in swaps:
        pair = frozenset((d, e))
        assert pair not in seen_pairs, f'pair {d}/{e} swapped twice'
        seen_pairs.add(pair)
        li, lj = launch.index(d), launch.index(e)
        assert (li < lj) == (trank[d] > trank[e]), \
            f'phantom swap {d}/{e} (not an inversion)'
    n = len(swaps)
    print(f'swaps ({n}): {swaps}')

    W = (x1 - x0) / (n + 1)
    first, last = {}, {}
    invol = {}
    for s, (d, p) in enumerate(swaps):
        first.setdefault(d, s)
        last[d] = s
        invol.setdefault(d, []).append(s)
        invol.setdefault(p, []).append(s)

    # windows: dive/surface vias want >= 0.45mm clear of the nearest swap
    # column, but must stay strictly inside the diver's own band region:
    # a swap involving d recorded during ANOTHER diver's move needs d on
    # F there, so the window is clamped away from those stages.
    window = {}
    for d in divers:
        if d in first:
            s1, sk = first[d], last[d]
            fw = max((s for s in invol[d] if s < s1), default=None)
            fe = min((s for s in invol[d] if s > sk), default=None)
            wa_lo = x0 + (fw + 1.5) * W + 0.10 if fw is not None \
                else x0 - 0.05
            wa = max(wa_lo, min(x0 + (s1 + 0.4) * W,
                                x0 + (s1 + 0.5) * W - 0.45))
            wb_hi = x0 + (fe + 0.5) * W - 0.10 if fe is not None \
                else x1 - 0.15
            wb = min(wb_hi, max(x0 + (sk + 1.6) * W,
                                x0 + (sk + 1.5) * W + 0.45))
            if fe is None and wb < x0 + (sk + 1.5) * W + 0.28:
                # no trunk room after the last own swap: surface on the
                # street run instead (via next to the field = cheap kind)
                wb = min(x1 + 0.31, ends[d][1][0] - 0.35)
        else:
            wa = x0 + (n + 0.75) * W
            if invol.get(d):
                wa = max(wa, x0 + (max(invol[d]) + 1.5) * W + 0.10)
            wb = x1 - 0.15
        if entry[d][0] == 'B':
            window[d] = (wa, None)      # None = extends to dogbone site
        else:
            window[d] = (wa, wb)
        print(f'{d}: window ({window[d][0]:.2f}, '
              f'{window[d][1] and round(window[d][1], 2)})')

    py = [lane_y[nm] for nm in target]
    assert all(b > a for a, b in zip(py, py[1:])), ('lane ys not strictly '
                                                    'increasing', py)
    Ly = sorted(ends[nm][0][1] for nm in names)

    def slot(t, i):
        return (1 - t) * Ly[i] + t * py[i]

    orders = [list(launch)]
    for (d, p) in swaps:
        o = list(orders[-1])
        i, j = o.index(d), o.index(p)
        o[i], o[j] = o[j], o[i]
        orders.append(o)

    # per-net geometry + layered segments + vias
    out_segs, out_vias = {}, {}
    for nm in names:
        mode, v = entry[nm]
        trunk = [ends[nm][0]]
        for s in range(n + 1):
            xm = x0 + (s + 0.5) * W
            t = (xm - x0) / (x1 - x0)
            trunk.append((xm, slot(t, orders[s].index(nm))))
        bx, by = ends[nm][1][0], ends[nm][1][1]
        if mode == 'F':
            sy = v
            path = trunk + [(x1, sy), (bx, sy), (bx, by)]
        elif mode in ('N', 'S'):
            sx, ry, ly, dx_ = v
            path = trunk + [(dx_, ly), (dx_, ry), (sx, ry),
                            (sx, by), (bx, by)]
        else:
            sx, sy = v
            path = trunk + [(x1, lane_y[nm]), (sx, sy)]
        path = [p for i, p in enumerate(path)
                if i == 0 or p != path[i - 1]]
        if nm not in window:
            out_segs[nm] = [(p, q, 'F.Cu') for p, q in zip(path, path[1:])]
            out_vias[nm] = []
            continue
        wa, wb = window[nm]
        aug, vias = [path[0]], []
        cuts = [xv for xv in (wa, wb) if xv is not None]
        for q in path[1:]:
            p = aug[-1]
            for xv in cuts:
                if p[0] < xv < q[0]:
                    tt = (xv - p[0]) / (q[0] - p[0])
                    vp = (xv, p[1] + tt * (q[1] - p[1]))
                    aug.append(vp)
                    vias.append(vp)
            aug.append(q)
        segs = []
        for p, q in zip(aug, aug[1:]):
            mx = (p[0] + q[0]) / 2
            on_b = wa < mx < wb if wb is not None else wa < mx
            segs.append((p, q, 'B.Cu' if on_b else 'F.Cu'))
        if mode == 'B':
            vias.append((v[0], v[1]))           # surface at dogbone site
            segs.append(((v[0], v[1]), (bx, by), 'F.Cu'))  # stub to ball
        out_segs[nm] = segs
        out_vias[nm] = vias

    obs_cache = {}

    def obs_for(nid, layer):
        if (nid, layer) not in obs_cache:
            obs_cache[(nid, layer)] = build_obstacles(pcb, nid, kids, layer)
        return obs_cache[(nid, layer)]

    def run_verify(quiet=False):
        hits = []
        bad = 0

        def note(msg, x, *nets):
            nonlocal bad
            bad += 1
            hits.append((nets, x))
            if not quiet:
                print(msg)

        for nm in names:
            nid, _ = byname[nm]
            for (p, q, layer) in out_segs[nm]:
                o = obs_for(nid, layer)
                if not o.seg_clear(p, q):
                    note(f'  STATIC HIT {nm} {layer} '
                         f'({p[0]:.2f},{p[1]:.2f})->'
                         f'({q[0]:.2f},{q[1]:.2f}) '
                         f'{sorted(o.hugs([p, q], slack=0.0))}',
                         (p[0] + q[0]) / 2, nm)
        min_cc = TRACK + 0.1
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                for (p, q, la) in out_segs[names[i]]:
                    for (c, e, lb) in out_segs[names[j]]:
                        if la != lb:
                            continue
                        d = ts.seg_seg_dist(p, q, c, e)
                        if d < min_cc:
                            note(f'  PAIR HIT {names[i]}/{names[j]} {la} '
                                 f'd={d:.3f} near ({p[0]:.2f},{p[1]:.2f})',
                                 (p[0] + q[0]) / 2, names[i], names[j])
        extra = (VIA_SIZE - TRACK) / 2
        for nm in names:
            nid, _ = byname[nm]
            for (vx, vy) in out_vias[nm]:
                for layer in ('F.Cu', 'B.Cu'):
                    vv = obs_for(nid, layer).point_violation((vx, vy),
                                                             pad=extra)
                    if vv and vv[0] > 0:
                        note(f'  VIA HIT {nm} {layer} '
                             f'({vx:.2f},{vy:.2f}) depth={vv[0]:.3f}',
                             vx, nm)
                for om in names:
                    if om == nm:
                        continue
                    for (p, q, _l) in out_segs[om]:
                        if ts.seg_pt_dist(p, q, (vx, vy)) < \
                                VIA_SIZE / 2 + TRACK / 2 + 0.1:
                            note(f'  VIA/TRACK HIT {nm} via '
                                 f'({vx:.2f},{vy:.2f}) vs {om}',
                                 vx, nm, om)
                    for (ox, oy) in out_vias[om]:
                        if math.hypot(vx - ox, vy - oy) < VIA_SIZE + 0.1:
                            if nm < om:
                                note(f'  VIA/VIA HIT {nm}/{om} '
                                     f'({vx:.2f},{vy:.2f})', vx, nm, om)
        return bad, hits

    # octilinearize with verifier-driven LOCAL refinement: only the
    # x-windows around a violation get a tighter deviation tube (the
    # octified path converges to the raw geometry there, which verified
    # clean); the rest of the drawing keeps the pretty 0.05 tube
    if not a.no_octi:
        raw_segs = {nm: list(out_segs[nm]) for nm in names}
        windows = {nm: [] for nm in names}
        fine_eps = 0.02
        for it in range(6):
            for nm in names:
                f = (windows[nm], fine_eps) if windows[nm] else None
                out_segs[nm] = octify_segs(raw_segs[nm], 0.05, f)
            bad, hits = run_verify(quiet=True)
            if not bad:
                break
            for nets_, x in hits:
                for nm in nets_:
                    windows[nm].append((x - 0.5, x + 0.5))
            print(f'  octify pass {it}: {bad} violations, refining '
                  f'{sorted(set(n for ns, _ in hits for n in ns))} '
                  f'at fine eps {fine_eps}')
            fine_eps = max(fine_eps / 2.5, 0.003)
        nw = sum(len(w) for w in windows.values())
        print(f'octilinearized: '
              f'{sum(len(s) for s in out_segs.values())} segments '
              f'({nw} fine windows)')
    if a.dump_segs:
        import json
        with open(a.dump_segs, 'w') as f:
            json.dump({nm: [[p, q, l] for (p, q, l) in out_segs[nm]]
                       for nm in names}, f)

    print('\nverification:')
    bad, _off = run_verify()
    print(f'  {"CLEAN" if not bad else str(bad) + " violations"}')

    # repo octolinear smoothing (#536): collapse the distributed 45
    # nudges into single elbows and merge our launch legs with the
    # fanout stubs. Every collapse is clearance-validated against ALL
    # copper and connectivity-guarded per net; it splices
    # pcb_data.segments in place, so the final geometry is read back
    # from there.
    smoothed = False
    if not a.no_smooth:
        from kicad_parser import Segment, Via
        from pcb_modification import smooth_octolinear_chains
        pre_len = {nm: sum(math.hypot(q[0] - p[0], q[1] - p[1])
                           for (p, q, _l) in out_segs[nm]) for nm in names}
        for nm in names:
            nid, _ = byname[nm]
            for (p, q, layer) in out_segs[nm]:
                pcb.segments.append(Segment(p[0], p[1], q[0], q[1],
                                            TRACK, layer, nid))
            for (vx, vy) in out_vias[nm]:
                pcb.vias.append(Via(vx, vy, VIA_SIZE, VIA_DRILL,
                                    ['F.Cu', 'B.Cu'], nid))
        _n, _nets, _rm, _addl, st = smooth_octolinear_chains(
            [], pcb, kids, clearance=0.1)
        final_segs = {}
        for nm in names:
            nid, _ = byname[nm]
            final_segs[nm] = [s for s in pcb.segments if s.net_id == nid]
        post_len = {nm: sum(math.hypot(s.end_x - s.start_x,
                                       s.end_y - s.start_y)
                            for s in final_segs[nm]) for nm in names}
        print(f'\nsmooth_octolinear_chains (#536): '
              f'{st.get("spans", 0)} spans on {_nets} nets, '
              f'-{st.get("saved_mm", 0):.2f} mm; segments '
              f'{sum(len(s) for s in out_segs.values())} -> '
              f'{sum(len(s) for s in final_segs.values())}; length '
              f'{sum(pre_len.values()):.2f} -> '
              f'{sum(post_len.values()):.2f} mm')
        smoothed = True

    # write board
    txt = open(a.board, encoding='utf-8').read()
    add = []
    if smoothed:
        txt = strip_net_segments(txt, kids)
        for nm in names:
            nid, _ = byname[nm]
            for s in final_segs[nm]:
                add.append(
                    f'  (segment (start {s.start_x:.4f} {s.start_y:.4f}) '
                    f'(end {s.end_x:.4f} {s.end_y:.4f}) '
                    f'(width {s.width}) '
                    f'(layer "{s.layer}") (net {nid}))\n')
            for (vx, vy) in out_vias[nm]:
                add.append(
                    f'  (via (at {vx:.4f} {vy:.4f}) (size {VIA_SIZE}) '
                    f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                    f'(net {nid}))\n')
            for s in final_segs[nm]:
                add.append(
                    f'  (gr_line (start {s.start_x:.4f} {s.start_y:.4f}) '
                    f'(end {s.end_x:.4f} {s.end_y:.4f}) '
                    f'(stroke (width 0.05) (type solid)) '
                    f'(layer "Eco2.User"))\n')
    else:
        for nm in names:
            nid, _ = byname[nm]
            for (p, q, layer) in out_segs[nm]:
                add.append(
                    f'  (segment (start {p[0]:.4f} {p[1]:.4f}) '
                    f'(end {q[0]:.4f} {q[1]:.4f}) (width {TRACK}) '
                    f'(layer "{layer}") (net {nid}))\n')
            for (vx, vy) in out_vias[nm]:
                add.append(
                    f'  (via (at {vx:.4f} {vy:.4f}) (size {VIA_SIZE}) '
                    f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                    f'(net {nid}))\n')
            for (p, q, _l) in out_segs[nm]:
                add.append(
                    f'  (gr_line (start {p[0]:.4f} {p[1]:.4f}) '
                    f'(end {q[0]:.4f} {q[1]:.4f}) '
                    f'(stroke (width 0.05) (type solid)) '
                    f'(layer "Eco2.User"))\n')
    k = txt.rstrip().rfind(')')
    out_board = a.out + '.kicad_pcb'
    with open(out_board, 'w') as f:
        f.write(txt[:k] + ''.join(add) + txt[k:])
    pro = os.path.splitext(a.board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, a.out + '.kicad_pro')
    nv = sum(len(v) for v in out_vias.values())
    nseg = sum(len(final_segs[nm]) for nm in names) if smoothed else \
        sum(len(s) for s in out_segs.values())
    print(f'\nwrote {out_board}: {nseg} segments, {nv} vias')


if __name__ == '__main__':
    main()
