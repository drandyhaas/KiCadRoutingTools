#!/usr/bin/env python3
"""pair_census.py BOARD [--nets a,b,...] [--gap 0.1] [--track 0.127] [--tol 0.06]

How a board's differential pairs are ROUTED, per pair, as numbers a judge
can price (#622, the pairs-in-the-bus work, 2026-09-20):

  * P and N routed length, their SKEW (|P - N|);
  * the COUPLED fraction: the share of P's copper that runs beside N on the
    same layer at the pair pitch (track + gap, within `tol`) -- 1.0 is a
    pair routed as a pair, ~0 is two singles that happen to share ends;
  * vias per leg and whether the two legs change layer at the same places
    (a pair dives together: two barrels side by side).

Pairs are found by NAME, generally: two nets whose names differ only in a
trailing P/N, _P/_N, +/-, p/n. Nothing here knows a board, a part or a bus.
`--nets` restricts the census to pairs whose BOTH legs are in the list (the
run's nets), as grade_k scopes its checks.
"""

KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # #937: a research tool (awx), catalogued, shown at no door
import argparse
import contextlib
import io
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(HERE, '..', 'py_router'))

_SUFFIX = re.compile(r'^(.*?)(_P|_N|P|N|\+|-|_p|_n)$')


def pair_names(names):
    """{base: (P name, N name)} over the given net names, by suffix."""
    by = {}
    for nm in names:
        m = _SUFFIX.match(nm)
        if not m:
            continue
        base, suf = m.group(1), m.group(2)
        if not base:
            continue
        pol = 'P' if suf in ('_P', 'P', '+', '_p') else 'N'
        by.setdefault(base, {})[pol] = nm
    return {b: (d['P'], d['N']) for b, d in by.items() if 'P' in d and 'N' in d}


def _seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def census(pcb, nets=None, track=0.127, gap=0.1, tol=0.06, step=0.05):
    short = lambda n: n.split('/')[-1]
    byname = {short(n.name): i for i, n in pcb.nets.items()}
    names = list(byname) if nets is None else [n for n in nets if n in byname]
    pairs = pair_names(names)
    pitch = track + gap
    out = {}
    for base, (pn, nn) in sorted(pairs.items()):
        pid, nid = byname[pn], byname[nn]
        ps = [g for g in pcb.segments if g.net_id == pid]
        ns = [g for g in pcb.segments if g.net_id == nid]
        pv = [v for v in pcb.vias if v.net_id == pid]
        nv = [v for v in pcb.vias if v.net_id == nid]
        lp = sum(math.hypot(g.end_x - g.start_x, g.end_y - g.start_y) for g in ps)
        ln = sum(math.hypot(g.end_x - g.start_x, g.end_y - g.start_y) for g in ns)
        # coupled fraction: sample P, measure the distance to N on the same
        # layer. The pair PITCH is read off the board, not assumed -- the
        # mode of that distance over the samples (a human's pair may run
        # at 0.35 mm, ours at track + gap) -- and a sample is coupled when
        # it sits within `tol` of the mode. `pitch` (track + gap) is only
        # the fallback when nothing runs parallel at all.
        n_by_layer = {}
        for g in ns:
            n_by_layer.setdefault(g.layer, []).append(g)
        samples = []          # (weight, distance or None)
        for g in ps:
            L = math.hypot(g.end_x - g.start_x, g.end_y - g.start_y)
            if L < 1e-6:
                continue
            k = max(1, int(L / step))
            near = n_by_layer.get(g.layer, [])
            for i in range(k):
                t = (i + 0.5) / k
                x = g.start_x + t * (g.end_x - g.start_x)
                y = g.start_y + t * (g.end_y - g.start_y)
                d = (min(_seg_dist(x, y, h.start_x, h.start_y, h.end_x, h.end_y) for h in near)
                     if near else None)
                samples.append((L / k, d))
        total = sum(w for w, _ in samples)
        hist = {}
        for w, d in samples:
            if d is not None and 0.12 <= d <= 1.2:
                hist[round(d / 0.02)] = hist.get(round(d / 0.02), 0.0) + w
        mode = (max(hist.items(), key=lambda kv: kv[1])[0] * 0.02) if hist else pitch
        coupled = sum(w for w, d in samples if d is not None and abs(d - mode) <= tol)
        # dives together: every P via has an N via within 2 pitches
        together = sum(1 for v in pv if any(math.hypot(v.x - w.x, v.y - w.y) <= 2.5 * pitch + v.size for w in nv))
        out[base] = {'P': pn, 'N': nn, 'len_p': lp, 'len_n': ln, 'skew': abs(lp - ln),
                     'coupled': (coupled / total) if total else 0.0, 'pitch': mode,
                     'vias_p': len(pv), 'vias_n': len(nv), 'vias_together': together,
                     'routed': lp > 0.5 and ln > 0.5}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('--nets', default='', help='comma-separated run nets (both legs must be listed)')
    ap.add_argument('--track', type=float, default=0.127)
    ap.add_argument('--gap', type=float, default=0.1)
    ap.add_argument('--tol', type=float, default=0.06)
    a = ap.parse_args(argv)
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(a.board)
    nets = [n for n in a.nets.split(',') if n] or None
    c = census(pcb, nets, a.track, a.gap, a.tol)
    if not c:
        print('no pairs by name' + (' among the run nets' if nets else ''))
        return 0
    print(f'{"pair":10s} {"P mm":>7s} {"N mm":>7s} {"skew":>6s} {"coupled":>8s} {"pitch":>6s} {"vias P/N":>9s} {"together":>8s}')
    for b, r in c.items():
        print(f'{b:10s} {r["len_p"]:7.2f} {r["len_n"]:7.2f} {r["skew"]:6.2f} {r["coupled"]:8.2f} {r["pitch"]:6.2f} '
              f'{r["vias_p"]:>4d}/{r["vias_n"]:<4d} {r["vias_together"]:>8d}'
              + ('' if r['routed'] else '   (not routed)'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
