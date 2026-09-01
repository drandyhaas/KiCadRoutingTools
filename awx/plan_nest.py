#!/usr/bin/env python3
"""#622 nesting model: same-page ride interleavings in the cyclic
boundary model, and the headroom HOMOTOPY adjustment unlocks.

Model: the routing field between two arrays, contracted to a disk.
Each array's perimeter is an arc of the disk boundary, CUT at the
array's BACK (the boundary point facing away from the other array --
the one place lanes do not pass); the two arcs are glued with
mirrored orientation so a bundle of parallel lanes reads as nested
(zero interleavings). Each net is a chord between its two ESCAPE
CROSSINGS -- where its copper actually crosses the package boundary,
NOT its ball: the escape crossing is exactly where a homotopy choice
is expressed in copper. Two same-page chords cross iff their ports
interleave on the glued cycle.

A ride may WRAP an array (around its north or south, arriving from
the back): the effective port slides to that end of the arc, berth
unmoved -- homotopy adjusted at either end. `analyze()` returns the
as-is crossing count and the wrap moves a local descent recommends;
the CLI prints them.

Calibration (measured 0901): the count is a COMPARATIVE signal, not
an absolute -- the human's end-dive pattern COVERS crossings placed
in a net's tail zones (their K28 board carries 63 interleavings at
46 vias), so zero is not the target; fewer trapped mid-field
crossings is. Ours vs human as-is: K28 32/63, K35 75/79, K41 111/96;
ours after homotopy descent: 9 / 25 / 32.

usage: plan_nest.py BOARD.kicad_pcb K [--src U1] [--dst DU1]
"""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

EPS = 1e-4


def _bbox(fp):
    xs = [p.global_x for p in fp.pads]
    ys = [p.global_y for p in fp.pads]
    return (min(xs) - 0.4, min(ys) - 0.4, max(xs) + 0.4, max(ys) + 0.4)


def _walk_t(x, y, bb, back_west, first_north):
    """Perimeter coordinate in [0,1): cut at the BACK face midpoint,
    walking so the first travel is northward (first_north) or south.
    The two arrays use mirrored senses so parallel lanes nest."""
    x0, y0, x1, y1 = bb
    px = min(max(x, x0), x1)
    py = min(max(y, y0), y1)
    d = {'N': py - y0, 'S': y1 - py, 'W': px - x0, 'E': x1 - px}
    side = min(d, key=d.get)
    w, h = x1 - x0, y1 - y0
    ymid = (y0 + y1) / 2
    P = 2 * (w + h)
    if back_west and first_north:
        # W-mid up -> N(W->E) -> E(N->S) -> S(E->W) -> W-mid
        if side == 'W':
            t = (ymid - py) if py <= ymid else P - (py - ymid)
        elif side == 'N':
            t = (ymid - y0) + (px - x0)
        elif side == 'E':
            t = (ymid - y0) + w + (py - y0)
        else:
            t = (ymid - y0) + w + h + (x1 - px)
    elif not back_west and not first_north:
        # E-mid down -> S(E->W) -> W(S->N) -> N(W->E) -> E-mid
        if side == 'E':
            t = (py - ymid) if py >= ymid else P - (ymid - py)
        elif side == 'S':
            t = (y1 - ymid) + (x1 - px)
        elif side == 'W':
            t = (y1 - ymid) + w + (y1 - py)
        else:
            t = (y1 - ymid) + w + h + (px - x0)
    else:
        raise ValueError('unsupported walk combination')
    return (t / P) % 1.0


def _escape_port(pcb, nid, ball, bb):
    """Where the net's copper crosses this array's inflated bbox --
    the crossing nearest the ball (the escape census rule). Falls
    back to the ball for a net with no crossing (unrouted stub)."""
    def inside(x, y):
        return bb[0] <= x <= bb[2] and bb[1] <= y <= bb[3]
    best = None
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        i0, i1 = inside(s.start_x, s.start_y), inside(s.end_x, s.end_y)
        if i0 == i1:
            continue
        (ix, iy), (ox, oy) = ((s.start_x, s.start_y),
                              (s.end_x, s.end_y)) if i0 else \
            ((s.end_x, s.end_y), (s.start_x, s.start_y))
        lo, hi = 0.0, 1.0
        for _ in range(40):
            t = (lo + hi) / 2
            if inside(ix + t * (ox - ix), iy + t * (oy - iy)):
                lo = t
            else:
                hi = t
        px, py = ix + lo * (ox - ix), iy + lo * (oy - iy)
        dball = math.hypot(px - ball.global_x, py - ball.global_y)
        if best is None or dball < best[0]:
            best = (dball, px, py)
    if best is None:
        return ball.global_x, ball.global_y
    return best[1], best[2]


def _crossings(assign):
    X = 0
    pairs = []
    names = sorted(assign)
    for i, m in enumerate(names):
        for n2 in names[i + 1:]:
            s1, d1, p1 = assign[m]
            s2, d2, p2 = assign[n2]
            if p1 != p2:
                continue
            ins = (s1 < s2 < d1) or (s1 < d2 < d1)
            both = (s1 < s2 < d1) and (s1 < d2 < d1)
            if ins and not both:
                X += 1
                pairs.append((m, n2))
    return X, pairs


def analyze(board, nets, src='U1', dst='DU1', allow_pages=False):
    """Nest analysis of one board, scoped to `nets` (short names).
    Returns a dict:
      as_is        same-page interleaving count, ports from copper
      pairs        the interleaved pairs
      minimum      count after the homotopy descent
      moves        {net: (tag, added_mm)} the descent recommends,
                   tag in src-wrapN/src-wrapS/dst-wrapN/dst-wrapS
                   (page flip tags appear only with allow_pages)
    """
    pcb = parse_kicad_pcb(board)
    short = {i: n.name.rsplit('/', 1)[-1] for i, n in pcb.nets.items()}
    sfp, dfp = pcb.footprints[src], pcb.footprints[dst]
    sbb, dbb = _bbox(sfp), _bbox(dfp)
    if (sbb[0] + sbb[2]) > (dbb[0] + dbb[2]):
        raise ValueError('analyze() expects src west of dst; extend '
                         'the walk table for other layouts')
    sper = 2 * ((sbb[2] - sbb[0]) + (sbb[3] - sbb[1]))
    dper = 2 * ((dbb[2] - dbb[0]) + (dbb[3] - dbb[1]))

    def ride_page(nid):
        tot = {}
        for s in pcb.segments:
            if s.net_id == nid:
                L = math.hypot(s.end_x - s.start_x,
                               s.end_y - s.start_y)
                tot[s.layer] = tot.get(s.layer, 0.0) + L
        return max(tot, key=tot.get) if tot else None

    R = {}
    want = set(nets)
    for nid, net in pcb.nets.items():
        nm = short.get(nid)
        if nm not in want:
            continue
        sball = next((p for p in sfp.pads if p.net_id == nid), None)
        dball = next((p for p in dfp.pads if p.net_id == nid), None)
        pg = ride_page(nid)
        if not sball or not dball or pg is None:
            continue
        sx, sy = _escape_port(pcb, nid, sball, sbb)
        dx, dy = _escape_port(pcb, nid, dball, dbb)
        R[nm] = {'s': _walk_t(sx, sy, sbb, True, True),
                 'd': 1.0 + _walk_t(dx, dy, dbb, False, False),
                 'page': pg}

    def opts(e):
        o = [('direct', e['s'], e['d'], 0.0)]
        for tag, t in (('src-wrapN', EPS), ('src-wrapS', 1.0 - EPS)):
            o.append((tag, t, e['d'], abs(t - e['s']) * sper))
        for tag, t in (('dst-wrapS', 1.0 + EPS),
                       ('dst-wrapN', 2.0 - EPS)):
            o.append((tag, e['s'], t, abs(t - e['d']) * dper))
        return o

    cur = {m: (e['s'], e['d'], e['page']) for m, e in R.items()}
    x0, pairs0 = _crossings(cur)
    st = dict(cur)
    wrapmm = {m: 0.0 for m in st}
    choice = {m: 'direct' for m in st}
    for _ in range(12):
        improved = False
        for m in sorted(st):
            best = (_crossings(st)[0], st[m], choice[m], wrapmm[m])
            for tag, sp, dp, mm in opts(R[m]):
                pages = [st[m][2]] if not allow_pages else \
                    ['F.Cu', 'B.Cu']
                for pg in pages:
                    trial = dict(st)
                    trial[m] = (sp, dp, pg)
                    x = _crossings(trial)[0]
                    if (x, mm) < (best[0], best[3]):
                        best = (x, (sp, dp, pg), tag, mm)
            if best[1] != st[m]:
                st[m] = best[1]
                choice[m] = best[2]
                wrapmm[m] = best[3]
                improved = True
        if not improved:
            break
    xmin, _ = _crossings(st)
    moves = {}
    for m in st:
        if choice[m] == 'direct' and st[m][2] == cur[m][2]:
            continue
        # solo gain: crossings removed by THIS move alone on the
        # as-is state -- the ranking a serial trial channel needs
        solo = dict(cur)
        solo[m] = st[m]
        moves[m] = (choice[m], wrapmm[m],
                    x0 - _crossings(solo)[0])
    return {'as_is': x0, 'pairs': pairs0, 'minimum': xmin,
            'moves': moves, 'rides': len(R)}


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k')
    ap.add_argument('--src', default='U1')
    ap.add_argument('--dst', default='DU1')
    a = ap.parse_args()
    nets = subprocess.run(
        [sys.executable, os.path.join(HERE, 'coherent_nets.py'), a.k],
        capture_output=True, text=True).stdout.strip().split(',')
    r = analyze(a.board, nets, a.src, a.dst)
    print(f'{os.path.basename(a.board)} K={a.k}: {r["rides"]} rides')
    print(f'as-is same-page interleavings: {r["as_is"]}')
    for m, n2 in r['pairs'][:14]:
        print(f'  {m} x {n2}')
    if len(r['pairs']) > 14:
        print(f'  ... +{len(r["pairs"]) - 14} more')
    print(f'homotopy-adjusted minimum: {r["minimum"]} '
          f'({len(r["moves"])} net(s) re-wrapped, '
          f'+{sum(v[1] for v in r["moves"].values()):.1f}mm est)')
    for m, (tag, mm, solo) in sorted(r['moves'].items(),
                                     key=lambda kv: -kv[1][2]):
        print(f'  {m}: {tag} (+{mm:.1f}mm, solo -{solo} crossings)')
    rp = analyze(a.board, nets, a.src, a.dst, allow_pages=True)
    print(f'homotopy+pages minimum:    {rp["minimum"]} '
          f'({len(rp["moves"])} move(s))')


if __name__ == '__main__':
    main()
