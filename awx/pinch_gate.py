#!/usr/bin/env python3
"""pinch_gate.py -- THE PLAN item D4's gate, per LANE: does a pinch census
read off the plan predict which lanes the braid REFUSES?

For every recorded run (a fanout board + its `.plan.json` + the braid's log
`<tag>_k<K>.log` + the routed board) the braid's plan phase gives each
lane its planned polyline and page; the census then counts, per lane:

    end_pinch    foreign fanout copper (other nets' stubs, vias, pads; any
                 layer) within VIA_NEED of the lane's two ends -- the via
                 sites (launch slot, landing slot)
    run_static   foreign SAME-layer fanout segments within PROX_TRACK of the
                 polyline (sampled every 0.2 mm) -- the static walls
    run_lanes    OTHER planned lanes on the SAME page within PROX_TRACK of
                 the polyline (sampled) -- the dynamic pinch by the virtual-
                 copper net, the blocker the refusal lines name
    xings        other lanes' polylines crossing this one on the same page
                 (an inversion the schedule must break with a change)

and the labels from the braid's log: refused at attempt 0 (in band, first
try), routed only at the LAST CALL (with its via cost), plus the routed
via count per net. Per K it prints each predictor's AUC for both labels
and its Spearman against the routed vias, over every lane of every run.
The bar (the review): beat room_probe's recorded null, AUC well above 0.5
on a held-out K, before anything prices a pinch.

Usage: pinch_gate.py [--roots tmp,tmp/s7,tmp/s8,tmp/s9,tmp/s10] [--ks 28,35,41,51]
                     [--workers 4] [--out tmp/s10/pinch_gate.tsv]
"""
import argparse
import contextlib
import glob
import io
import json
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

FIELDS = ['dir', 'tag', 'K', 'net', 'page', 'swim', 'end_pinch', 'run_static', 'run_lanes', 'xings',
          'lane_mm', 'refused0', 'lastcall', 'lastcall_vias', 'routed']


def coherent(K):
    r = subprocess.run([sys.executable, 'coherent_nets.py', str(K), '--board=fb_t2q_fresh.kicad_pcb'],
                       capture_output=True, text=True)
    return [n for n in r.stdout.strip().split(',') if n]


def labels_of(log_path):
    """{net: (refused at attempt 0, routed at the last call, last-call vias)}."""
    out = {}
    attempt = None
    if not os.path.isfile(log_path):
        return out
    with open(log_path, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = re.match(r'\s*attempt (\d+):', line)
            if m:
                attempt = int(m.group(1))
                continue
            m = re.match(r'\s*refused: (\S+)', line)
            if m and attempt == 0:
                r0, lc, lv = out.get(m.group(1), (0, 0, 0))
                out[m.group(1)] = (1, lc, lv)
                continue
            m = re.match(r'\s*last call routed: (\S+) \((\d+) via', line)
            if m:
                r0, lc, lv = out.get(m.group(1), (0, 0, 0))
                out[m.group(1)] = (r0, 1, int(m.group(2)))
    return out


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def samples(pts, step=0.2):
    out = []
    for a, b in zip(pts, pts[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(L / step))
        for i in range(n):
            t = i / n
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    if pts:
        out.append(tuple(pts[-1]))
    return out


def _ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])


def crosses(p, q):
    n = 0
    for a, b in zip(p, p[1:]):
        for c, d in zip(q, q[1:]):
            if (_ccw(a, c, d) != _ccw(b, c, d)) and (_ccw(a, b, c) != _ccw(a, b, d)):
                n += 1
    return n


def grade_one(job):
    d, tag, K, names = job
    fo = os.path.join(d, f'{tag}_fo_k{K}.kicad_pcb')
    rt = os.path.join(d, f'{tag}_k{K}.kicad_pcb')
    import braid as te
    from kicad_parser import parse_kicad_pcb
    try:
        plan = json.load(open(fo.replace('.kicad_pcb', '.plan.json')))
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            bp = te.plan_braid(fo, names, 'DU1', plan)
        fpcb = parse_kicad_pcb(fo)
        rpcb = parse_kicad_pcb(rt)
    except Exception as e:
        return [{'dir': d, 'tag': tag, 'K': K, 'error': f'{type(e).__name__}: {e}'}]
    byname = {n.name.split('/')[-1]: i for i, n in fpcb.nets.items()}
    ids = {byname[nm]: nm for nm in names if nm in byname}
    rbyname = {n.name.split('/')[-1]: i for i, n in rpcb.nets.items()}
    routed = {nm: sum(1 for v in rpcb.vias if v.net_id == rbyname.get(nm, -1)) for nm in names}
    lab = labels_of(os.path.join(d, f'{tag}_k{K}.log'))
    # the foreign copper on the fanout board: every segment / via / pad not of the lane's net
    segs = [(s.net_id, s.layer, (s.start_x, s.start_y), (s.end_x, s.end_y)) for s in fpcb.segments]
    pts_any = [(v.net_id, (v.x, v.y)) for v in fpcb.vias]
    pts_any += [(p.net_id, (p.global_x, p.global_y)) for fp in fpcb.footprints.values() for p in fp.pads]
    lanes = {nm: (bp.get(nm, {}).get('lane') or [], bp.get(nm, {}).get('page')) for nm in names}
    rows = []
    for nm in names:
        pts, page = lanes[nm]
        nid = byname.get(nm)
        b = bp.get(nm, {})
        row = {'dir': d, 'tag': tag, 'K': K, 'net': nm, 'page': (page or 'swim')[0], 'swim': int(page is None),
               'end_pinch': 0, 'run_static': 0, 'run_lanes': 0, 'xings': 0, 'lane_mm': 0.0,
               'refused0': lab.get(nm, (0, 0, 0))[0], 'lastcall': lab.get(nm, (0, 0, 0))[1],
               'lastcall_vias': lab.get(nm, (0, 0, 0))[2], 'routed': routed.get(nm, 0)}
        if pts and len(pts) >= 2:
            row['lane_mm'] = round(sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:])), 1)
            ends = [tuple(pts[0]), tuple(pts[-1])]
            ep = 0
            for e in ends:
                for (onid, pt) in pts_any:
                    if onid != nid and math.hypot(pt[0] - e[0], pt[1] - e[1]) < te.VIA_NEED:
                        ep += 1
                for (onid, _L, a, bb) in segs:
                    if onid != nid and seg_dist(e, a, bb) < te.VIA_NEED:
                        ep += 1
            row['end_pinch'] = ep
            sm_ = samples(pts)
            rs = 0
            for (onid, L, a, bb) in segs:
                if onid == nid or L != page:
                    continue
                if any(seg_dist(p, a, bb) < te.PROX_TRACK for p in sm_[::2]):
                    rs += 1
            row['run_static'] = rs
            rl = xg = 0
            for om, (opts, opage) in lanes.items():
                if om == nm or not opts or len(opts) < 2 or opage != page:
                    continue
                if any(seg_dist(p, a, bb) < te.PROX_TRACK for p in sm_[::3] for a, bb in zip(opts, opts[1:])):
                    rl += 1
                xg += crosses(pts, opts)
            row['run_lanes'] = rl
            row['xings'] = xg
        rows.append(row)
    return rows


def auc(scores, labels):
    """The AUC of `scores` for the binary `labels` (ties at 0.5)."""
    pos = [s for s, l in zip(scores, labels) if l]
    neg = [s for s, l in zip(scores, labels) if not l]
    if not pos or not neg:
        return float('nan')
    tot = 0.0
    for p in pos:
        for n in neg:
            tot += 1.0 if p > n else (0.5 if p == n else 0.0)
    return tot / (len(pos) * len(neg))


def spearman(xs, ys):
    n = len(xs)
    if n < 3:
        return float('nan')

    def rk(v):
        order = sorted(range(n), key=lambda i: v[i])
        out = [0.0] * n
        i = 0
        while i < n:
            j = i
            while j + 1 < n and v[order[j + 1]] == v[order[i]]:
                j += 1
            for k in range(i, j + 1):
                out[order[k]] = (i + j) / 2 + 1
            i = j + 1
        return out
    rx, ry = rk(xs), rk(ys)
    mx, my = sum(rx) / n, sum(ry) / n
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    den = math.sqrt(sum((a - mx) ** 2 for a in rx) * sum((b - my) ** 2 for b in ry))
    return num / den if den else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--roots', default='tmp,tmp/s7,tmp/s8,tmp/s9,tmp/s10')
    ap.add_argument('--ks', default='28,35,41,51')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--out', default='tmp/s10/pinch_gate.tsv')
    a = ap.parse_args()
    ks = [int(k) for k in a.ks.split(',') if k]
    rows = []
    done = set()
    if os.path.isfile(a.out):
        with open(a.out, encoding='utf-8') as f:
            hdr = None
            for line in f:
                parts = line.rstrip('\n').split('\t')
                if hdr is None:
                    hdr = parts
                    continue
                r = dict(zip(hdr, parts))
                for k in ('K', 'swim', 'end_pinch', 'run_static', 'run_lanes', 'xings', 'refused0', 'lastcall',
                          'lastcall_vias', 'routed'):
                    r[k] = int(r[k])
                r['lane_mm'] = float(r['lane_mm'])
                rows.append(r)
                done.add((r['dir'], r['tag'], r['K']))
    names_of = {K: coherent(K) for K in ks}
    jobs = []
    for d in a.roots.split(','):
        for p in sorted(glob.glob(os.path.join(d, '*_fo_k*.plan.json'))):
            m = re.match(r'(.+)_fo_k(\d+)\.plan\.json$', os.path.basename(p))
            if not m:
                continue
            tag, K = m.group(1), int(m.group(2))
            if K not in ks or (d, tag, K) in done:
                continue
            fo = os.path.join(d, f'{tag}_fo_k{K}.kicad_pcb')
            rt = os.path.join(d, f'{tag}_k{K}.kicad_pcb')
            lg = os.path.join(d, f'{tag}_k{K}.log')
            if os.path.isfile(fo) and os.path.isfile(rt) and os.path.isfile(lg):
                jobs.append((d, tag, K, names_of[K]))
    print(f'{len(done)} cached run(s), {len(jobs)} to grade', flush=True)
    if jobs:
        if a.workers > 1:
            from multiprocessing import Pool
            with Pool(a.workers) as pool:
                results = list(pool.imap_unordered(grade_one, jobs))
        else:
            results = [grade_one(j) for j in jobs]
        for rs in results:
            for r in rs:
                if 'error' in r:
                    print(f'  {r["dir"]}/{r["tag"]} K{r["K"]}: {r["error"]}', flush=True)
                else:
                    rows.append(r)
        os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
        with open(a.out, 'w', encoding='utf-8') as f:
            f.write('\t'.join(FIELDS) + '\n')
            for r in rows:
                f.write('\t'.join(str(r.get(k, '')) for k in FIELDS) + '\n')
        print(f'{len(rows)} lane row(s) -> {a.out}', flush=True)
    preds = ['end_pinch', 'run_static', 'run_lanes', 'xings', 'lane_mm']
    for K in ks + [None]:
        rs = [r for r in rows if (K is None or r['K'] == K) and not r['swim']]
        if not rs:
            continue
        n0 = sum(r['refused0'] for r in rs)
        nl = sum(r['lastcall'] for r in rs)
        print(f'\n=== {"ALL" if K is None else "K" + str(K)}: {len(rs)} page lane(s), refused at attempt 0: {n0}, '
              f'last call: {nl}')
        print(f'  {"predictor":10s} {"AUC(ref0)":>9s} {"AUC(last)":>9s} {"rho(vias)":>9s}')
        for pn in preds:
            sc = [r[pn] for r in rs]
            print(f'  {pn:10s} {auc(sc, [r["refused0"] for r in rs]):9.2f} {auc(sc, [r["lastcall"] for r in rs]):9.2f} '
                  f'{spearman(sc, [r["routed"] for r in rs]):9.2f}')
        comb = [r['end_pinch'] + r['run_lanes'] + r['xings'] for r in rs]
        print(f'  {"end+lanes+x":10s} {auc(comb, [r["refused0"] for r in rs]):9.2f} '
              f'{auc(comb, [r["lastcall"] for r in rs]):9.2f} {spearman(comb, [r["routed"] for r in rs]):9.2f}')


if __name__ == '__main__':
    main()
