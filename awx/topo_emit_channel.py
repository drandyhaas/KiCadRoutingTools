#!/usr/bin/env python3
"""Octilinear channel emitter (v4) for the #622 topo-string experiment.

Pipeline: endpoints from the stub census; field entries assigned first
(one-track-per-street capacity on F.Cu, direct entries for column-A
balls, B.Cu dogbones for the overflow -- B.Cu is EMPTY under an SMD ball
field); lane rank = ENTRY y; divers = complement of the LIS of the
launch->entry permutation.

Trunk drawing is a CHANNEL: fixed horizontal rails at the entry ys, and
each net makes exactly ONE vertical hop from its tooth level to its
rail, at its own column. All crossings are vertical-through-horizontal
(90 degrees). A diver's hop is on B.Cu (dive via on its tooth stub --
at-pad, the cheap kind -- surface via on its rail); keeps hop on F.Cu
east of every diver column, so a keep's vertical only ever crosses
diver rails (B) and diver hop columns never touch keep rails that
matter. Layer legality reduces to via-position window constraints,
solved per candidate column order; the internal verifier (static
obstacles per layer + pairwise same-layer clearance + via checks) gates
candidates and the first clean one is emitted.

Geometry is octilinear-native: 0/90 trunk, streets horizontal, ball
jogs vertical, dogbone stubs exact 45, B entry diagonals decomposed
into 45 + axis runs.

Emits copper (segments + vias) into a board copy for check_connected /
check_drc grading, plus an Eco2 overlay of the same geometry.
"""
import argparse
import math
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
COL_PITCH = 0.55
EPS = 0.03


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


def octi45(p, q):
    """Waypoints from p to q using one 45 segment + one axis segment
    (p, q included)."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    if abs(dx) < 1e-9 or abs(dy) < 1e-9 or abs(abs(dx) - abs(dy)) < 1e-9:
        return [p, q]
    sx, sy = math.copysign(1, dx), math.copysign(1, dy)
    if abs(dx) > abs(dy):        # 45 first, then horizontal
        mid = (p[0] + sx * abs(dy), q[1])
    else:                        # 45 first, then vertical
        mid = (q[0], p[1] + sy * abs(dx))
    return [p, mid, q]


def verify(names, byname, kids, pcb, out_segs, out_vias, obs_cache,
           verbose=False):
    def obs_for(nid, layer):
        if (nid, layer) not in obs_cache:
            obs_cache[(nid, layer)] = build_obstacles(pcb, nid, kids, layer)
        return obs_cache[(nid, layer)]

    bad = 0
    for nm in names:
        nid, _ = byname[nm]
        for (p, q, layer) in out_segs[nm]:
            o = obs_for(nid, layer)
            if not o.seg_clear(p, q):
                if verbose:
                    print(f'  STATIC HIT {nm} {layer} '
                          f'({p[0]:.2f},{p[1]:.2f})->'
                          f'({q[0]:.2f},{q[1]:.2f}) '
                          f'{sorted(o.hugs([p, q], slack=0.0))}')
                bad += 1
    min_cc = TRACK + 0.1
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            for (p, q, la) in out_segs[names[i]]:
                for (c, e, lb) in out_segs[names[j]]:
                    if la != lb:
                        continue
                    d = ts.seg_seg_dist(p, q, c, e)
                    if d < min_cc:
                        if verbose:
                            print(f'  PAIR HIT {names[i]}/{names[j]} '
                                  f'{la} d={d:.3f} near '
                                  f'({p[0]:.2f},{p[1]:.2f})')
                        bad += 1
    extra = (VIA_SIZE - TRACK) / 2
    for nm in names:
        nid, _ = byname[nm]
        for (vx, vy) in out_vias[nm]:
            for layer in ('F.Cu', 'B.Cu'):
                vv = obs_for(nid, layer).point_violation((vx, vy),
                                                         pad=extra)
                if vv and vv[0] > 0:
                    if verbose:
                        print(f'  VIA HIT {nm} {layer} '
                              f'({vx:.2f},{vy:.2f}) '
                              f'depth={vv[0]:.3f}')
                    bad += 1
            for om in names:
                if om == nm:
                    continue
                for (p, q, _l) in out_segs[om]:
                    if ts.seg_pt_dist(p, q, (vx, vy)) < \
                            VIA_SIZE / 2 + TRACK / 2 + 0.1:
                        if verbose:
                            print(f'  VIA/TRACK HIT {nm} via '
                                  f'({vx:.2f},{vy:.2f}) vs {om}')
                        bad += 1
                for (ox, oy) in out_vias[om]:
                    if math.hypot(vx - ox, vy - oy) < VIA_SIZE + 0.1:
                        if nm < om:
                            if verbose:
                                print(f'  VIA/VIA HIT {nm}/{om} '
                                      f'({vx:.2f},{vy:.2f})')
                            bad += 1
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', default=os.path.join(HERE,
                    'fb_t2q_base.kicad_pcb'))
    ap.add_argument('--nets', default='SDQ15,SDQ14,SDQ13,SDQ11')
    ap.add_argument('--out', default=os.path.join(HERE, 'topo_k4_emit'))
    ap.add_argument('--pitch', type=float, default=0.8)
    a = ap.parse_args()
    names = [n.strip() for n in a.nets.split(',') if n.strip()]

    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    kids = {byname[nm][0] for nm in names}
    ends = endpoints(pcb, names, byname)

    comps = {ends[nm][2] for nm in names}
    fx0 = min(p.global_x for c in comps for p in pcb.footprints[c].pads)
    x1 = fx0 - a.pitch * 0.8
    x0 = max(ends[nm][0][0] for nm in names) + 0.05

    launch = sorted(names, key=lambda nm: ends[nm][0][1])
    print(f'launch order: {launch}')

    obsF = {nm: build_obstacles(pcb, byname[nm][0], kids, 'F.Cu')
            for nm in names}
    half = a.pitch / 2
    seg_min = TRACK + CLEAR
    via_seg_min = VIA_SIZE / 2 + TRACK / 2 + CLEAR

    # ---- entry assignment (unchanged from v3) ----
    entry = {}
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

    ball_order = sorted(names, key=lambda nm: (ends[nm][1][1],
                                               ends[nm][1][0]))
    for nm in ball_order:
        bx, by = ends[nm][1][0], ends[nm][1][1]
        cands = []
        if bx <= fx0 + 0.01:
            cands.append((by, [((x1, by), (bx, by))]))
        for sy in (by - half, by + half):
            cands.append((sy, [((x1, sy), (bx, sy)),
                               ((bx, sy), (bx, by))]))
        for ey, segs in cands:
            if any(abs(ey - e[1]) < 0.3 for e in entry.values()
                   if e[0] == 'F'):
                continue
            if f_ok(nm, segs):
                entry[nm] = ('F', ey)
                placed_f.extend(s for s in segs if s[0] != s[1])
                break

    for nm in ball_order:
        if nm in entry:
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

    f_sorted = sorted((e[1], nm) for nm, e in entry.items() if e[0] == 'F')
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
    print(f'divers ({len(divers)}): '
          f'{sorted(divers, key=lambda n: trank[n])}')
    for nm in target:
        mode, v = entry[nm]
        print(f'{nm}: {mode}-entry {v} lane_y={lane_y[nm]:.3f}'
              + (' (PROMOTED +2 vias)' if nm in promoted else ''))

    # ---- octilinear channel drawing ----
    tooth = {nm: ends[nm][0] for nm in names}
    rail = dict(lane_y)
    dlist = sorted(divers, key=lambda nm: trank[nm])
    klist = [nm for nm in names if nm not in divers]

    def spans_cross(mover, y):
        lo, hi = sorted((tooth[mover][1], rail[mover]))
        return lo + EPS < y < hi - EPS

    def build(diver_order, keep_order, obs_for):
        """Return (out_segs, out_vias, col, info) or (None, reason)."""
        order = list(diver_order) + list(keep_order)
        col = {}
        dive_vx = {}
        xc = x0 + 0.45
        placed_v = []
        vpad = (VIA_SIZE - TRACK) / 2

        def via_spot(nm, x, y, xmax):
            def blocked(x_):
                if any(math.hypot(x_ - px, y - py) < VIA_SIZE + 0.12
                       for (px, py) in placed_v):
                    return True
                for layer in ('F.Cu', 'B.Cu'):
                    vv = obs_for(byname[nm][0], layer).point_violation(
                        (x_, y), pad=vpad)
                    if vv and vv[0] > 0:
                        return True
                return False
            while blocked(x):
                x += 0.12
                if x > xmax:
                    return None
            placed_v.append((x, y))
            return x

        for nm in order:
            if nm in divers:
                # dive via first (column follows it east): east of every
                # earlier diver column whose hop crosses this tooth level
                # (nm must be F there), else right at the tooth
                f_req = [col[e] for e in diver_order
                         if e in col and spans_cross(e, tooth[nm][1])]
                vx = max(f_req) + 0.30 if f_req else tooth[nm][0] + 0.18
                vx = via_spot(nm, vx, tooth[nm][1], x1 - 0.8)
                if vx is None:
                    return None, f'no dive-via room for {nm}'
                dive_vx[nm] = vx
                xc = max(xc, vx + 0.25)
            lo, hi = sorted((tooth[nm][1], rail[nm]))
            layer = 'B.Cu' if nm in divers else 'F.Cu'
            o = obs_for(byname[nm][0], layer)
            while xc < x1 - 0.55 and not o.seg_clear((xc, lo), (xc, hi)):
                xc += 0.12       # slide past foreign copper on hop layer
            if xc >= x1 - 0.55:
                return None, f'no clear column for {nm}'
            col[nm] = xc
            xc += COL_PITCH
        vias = {}
        for d in diver_order:
            vx = dive_vx[d]
            if vx > col[d] - 0.22:
                return None, f'dive via crowds column for {d}'
            if entry[d][0] == 'B':
                # B to the dogbone site; nothing east may need it F
                lat = [e for e in order if col[e] > col[d]
                       and spans_cross(e, rail[d]) and e in divers]
                if lat:
                    return None, f'{d} lane crossed by diver {lat}'
                vias[d] = (vx, None)
                continue
            # surface via on the rail: before later DIVER hops crossing
            # the rail (d must be F), after later KEEP hops crossing it
            # (d must still be B) -- both at once is infeasible
            div_x = [col[e] for e in diver_order
                     if col[e] > col[d] and spans_cross(e, rail[d])]
            keep_x = [col[e] for e in keep_order
                      if spans_cross(e, rail[d])]
            if div_x and keep_x:
                return None, f'{d} rail needs F and B east'
            if keep_x:
                sx = via_spot(d, min(x1 + 0.31, ends[d][1][0] - 0.35),
                              rail[d], ends[d][1][0] - 0.35)
            else:
                xmax = min(div_x) - 0.25 if div_x else x1 - 0.2
                sx = via_spot(d, col[d] + 0.25, rail[d], xmax)
            if sx is None:
                return None, f'no surface-via room for {d}'
            vias[d] = (vx, sx)
        # keep hops must not cross other keeps' F geometry: tooth
        # horizontals of later keeps, rails of earlier keeps
        for i, k in enumerate(keep_order):
            for k2 in keep_order[i + 1:]:
                if spans_cross(k, tooth[k2][1]):
                    return None, f'keep {k} hop crosses {k2} tooth'
            for k2 in keep_order[:i]:
                if spans_cross(k, rail[k2]):
                    return None, f'keep {k} hop crosses {k2} rail'
        # geometry
        out_segs, out_vias = {}, {}
        for nm in names:
            ty = tooth[nm][1]
            ry = rail[nm]
            pts = [tooth[nm], (col[nm], ty)]
            if abs(ry - ty) > 1e-6:
                pts.append((col[nm], ry))
            mode, v = entry[nm]
            if mode == 'F':
                bx, by = ends[nm][1]
                pts.append((bx, ry))
                if abs(by - ry) > 1e-6:
                    pts.append((bx, by))
            else:
                sx2, sy2 = v
                pts.append((x1, ry))
                pts.extend(octi45((x1, ry), (sx2, sy2))[1:])
            pts = [p for i, p in enumerate(pts) if i == 0 or
                   abs(p[0] - pts[i - 1][0]) + abs(p[1] - pts[i - 1][1])
                   > 1e-9]
            if nm not in vias:
                out_segs[nm] = [(p, q, 'F.Cu')
                                for p, q in zip(pts, pts[1:])]
                out_vias[nm] = []
                continue
            wa, wb = vias[nm]
            aug, vlist = [pts[0]], []
            cuts = [xv for xv in (wa, wb) if xv is not None]
            for q in pts[1:]:
                p = aug[-1]
                for xv in cuts:
                    if p[0] < xv < q[0]:
                        tt = (xv - p[0]) / (q[0] - p[0])
                        vp = (xv, p[1] + tt * (q[1] - p[1]))
                        aug.append(vp)
                        vlist.append(vp)
                aug.append(q)
            segs = []
            for p, q in zip(aug, aug[1:]):
                mx = (p[0] + q[0]) / 2
                on_b = wa < mx < wb if wb is not None else wa < mx
                segs.append((p, q, 'B.Cu' if on_b else 'F.Cu'))
            if entry[nm][0] == 'B':
                sx2, sy2 = entry[nm][1]
                vlist.append((sx2, sy2))
                segs.extend((p, q, 'F.Cu') for p, q in zip(
                    octi45((sx2, sy2), ends[nm][1]),
                    octi45((sx2, sy2), ends[nm][1])[1:]))
            out_segs[nm] = segs
            out_vias[nm] = vlist
        return (out_segs, out_vias, col), None

    # candidate column orders, gated by the verifier
    dy = {nm: abs(rail[nm] - tooth[nm][1]) for nm in names}
    diver_orders = [sorted(dlist, key=lambda n: -dy[n]),
                    sorted(dlist, key=lambda n: dy[n]),
                    dlist, list(reversed(dlist))]
    keep_orders = [sorted(klist, key=lambda n: rail[n]),
                   sorted(klist, key=lambda n: -rail[n])]
    obs_cache = {}

    def obs_for(nid, layer):
        if (nid, layer) not in obs_cache:
            obs_cache[(nid, layer)] = build_obstacles(pcb, nid, kids, layer)
        return obs_cache[(nid, layer)]

    chosen = None
    attempt = 0
    for do in diver_orders:
        for ko in keep_orders:
            built, why = build(do, ko, obs_for)
            if built is None:
                print(f'  order {do}+{ko}: {why}')
                continue
            out_segs, out_vias, col = built
            bad = verify(names, byname, kids, pcb, out_segs, out_vias,
                         obs_cache, verbose=(attempt == 0))
            attempt += 1
            if bad == 0:
                chosen = (do, ko, out_segs, out_vias, col)
                break
            print(f'  order {do}+{ko}: {bad} violations')
        if chosen:
            break
    assert chosen, 'no candidate order verified clean'
    do, ko, out_segs, out_vias, col = chosen
    print(f'\ncolumn order: divers {do} then keeps {ko}')
    for nm in do + ko:
        print(f'  {nm}: hop at x={col[nm]:.2f} '
              f'({tooth[nm][1]:.3f} -> {rail[nm]:.3f})')
    print('verification: CLEAN (gated)')
    verify(names, byname, kids, pcb, out_segs, out_vias, obs_cache,
           verbose=True)

    # write board
    txt = open(a.board, encoding='utf-8').read()
    add = []
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
    print(f'\nwrote {out_board}: '
          f'{sum(len(s) for s in out_segs.values())} segments, {nv} vias')


if __name__ == '__main__':
    main()
