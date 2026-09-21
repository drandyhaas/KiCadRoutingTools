#!/usr/bin/env python3
"""judge_gate.py -- THE PLAN item 1's gate: does a candidate JUDGE order the
recorded plans the way the routed boards came out?

For every recorded chain run under the given roots (a fanout board
`<tag>_fo_k<K>.kicad_pcb` with its `.plan.json` sidecar, and the routed
board `<tag>_k<K>.kicad_pcb` beside it) the braid's plan phase is run on
the fanout board (braid.plan_braid, exactly what judge_by_braid calls) and
several counts are formed from its answer:

    resid   the braid's residue: lanes it gives no page (swimmers)
    model   the pages-first model's own via count (the fo log's last
            `model vias N`) -- the CURRENT judge under PLAN_PAGES is
            (resid, model)
    c_sw    ends AS LAID on the fanout board + every page lane's plan-
            implied `changes` + every swimmer's `swim_changes` +
            `cross_vias` (band_over at the swimmer price) -- THE PLAN's A
    c_flat  the same with a FLAT prices.SWIM per swimmer (session 4's
            pred_vs_routed finding: swim_changes over-predicts 2-3x)
    c_sw0 / c_flat0   the two without cross_vias

and each is held against the routed board: vias on the run's nets, and
the open nets among them. Per K it prints Spearman's rho against the
routed count (clean boards) and against routed + OPEN_PENALTY per open
net, and the pairwise concordance over pairs whose routed counts differ
by at least PAIR_MIN -- the fraction of such pairs the judge orders the
same way, and the fraction it orders the WRONG way by more than MARGIN
(a false accept: the judge would have taken the worse plan).

Usage: judge_gate.py [--roots tmp,tmp/s7,tmp/s8,tmp/s9] [--ks 28,35,41,51]
                     [--workers 4] [--out FILE.tsv] [--only TAG,TAG]
                     [--margin 0] [--pair-min 5] [--open-penalty 20]
Rows are cached in the --out file: a later run re-reads it and only
grades boards it has not seen (delete the file to regrade everything).
"""

KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # #937: a research tool (awx), catalogued, shown at no door
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

FIELDS = ['dir', 'tag', 'K', 'n', 'resid', 'model', 'pred_flat', 'ends', 'changes',
          'swim_ch', 'cross', 'routed', 'open', 'opens', 'swimmers',
          'lane_mm', 'nolane', 'routed_mm', 'swim_list']
VIA_MM = 7.5          # select_moves.VIA_MM: the one exchange rate (Andy, 2026-09-15: everywhere)
OTHER = re.compile(r'^(arc|rp|pf|bs|sw51|xing51|sl_chk|stamp|vc|am_chk|bis_)')   # arms whose BRAID or planner differed


def coherent(K):
    r = subprocess.run([sys.executable, 'coherent_nets.py', str(K),
                        '--board=fb_t2q_fresh.kicad_pcb'],
                       capture_output=True, text=True)
    return [n for n in r.stdout.strip().split(',') if n]


def vias_per_net(pcb, names):
    byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}
    return {nm: sum(1 for v in pcb.vias if v.net_id == byname.get(nm, -1)) for nm in names}


def mm_per_net(pcb, names):
    byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}
    ids = {byname[nm]: nm for nm in names if nm in byname}
    out = {nm: 0.0 for nm in names}
    for s in pcb.segments:
        nm = ids.get(s.net_id)
        if nm is not None:
            out[nm] += math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
    return out


def lane_mm(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def open_nets(board, names):
    r = subprocess.run([sys.executable, '../py_router/check_connected.py', board],
                       capture_output=True, text=True)
    opens = []
    for line in (r.stdout + r.stderr).splitlines():
        m = re.search(r'(\S+) \(net \d+\):', line)
        if m and m.group(1).split('/')[-1] in names:
            opens.append(m.group(1).split('/')[-1])
        m2 = re.match(r'\s+(\S+) \(\d+ pads?\)\s*$', line)
        if m2 and m2.group(1).split('/')[-1] in names:
            opens.append(m2.group(1).split('/')[-1])
    return sorted(set(opens))


def log_numbers(fo_log):
    """(model vias of the LAST pages-first solve, judge_by_braid's flat
    predicted total) from the fanout log, or None where absent."""
    model = pred = None
    if not os.path.isfile(fo_log):
        return model, pred
    with open(fo_log, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = re.search(r'pages-first: model vias (\d+)', line)
            if m:
                model = int(m.group(1))
            m = re.search(r'plan model total predicted vias: ([\d.]+)', line)
            if m:
                pred = float(m.group(1))
    return model, pred


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
        ends = vias_per_net(parse_kicad_pcb(fo), names)
        rpcb = parse_kicad_pcb(rt)
        routed = vias_per_net(rpcb, names)
        routed_mm = mm_per_net(rpcb, names)
        opens = open_nets(rt, names)
    except Exception as e:                     # a broken run is reported, not scored
        return {'dir': d, 'tag': tag, 'K': K, 'error': f'{type(e).__name__}: {e}'}
    resid = ch = sw = xv = 0
    swimmers, swim_list = [], []
    lmm = 0.0
    nolane = 0
    for nm in names:
        b = bp.get(nm, {})
        xv += b.get('cross_vias', 0) or 0
        if b.get('lane'):
            lmm += lane_mm(b['lane'])
        else:
            nolane += 1
        if b.get('page') is None:
            resid += 1
            sw += b.get('swim_changes') or 0
            swimmers.append(nm)
            swim_list.append(str(b.get('swim_changes')))
        else:
            ch += b.get('changes') or 0
    model, pred = log_numbers(os.path.join(d, f'{tag}_fo_k{K}.log'))
    return {'dir': d, 'tag': tag, 'K': K, 'n': len(names), 'resid': resid,
            'model': model, 'pred_flat': pred, 'ends': sum(ends.values()),
            'changes': ch, 'swim_ch': sw, 'cross': xv,
            'routed': sum(routed.values()), 'open': len(opens),
            'opens': ','.join(opens), 'swimmers': ','.join(swimmers),
            'lane_mm': round(lmm, 1), 'nolane': nolane, 'routed_mm': round(sum(routed_mm.values()), 1),
            'swim_list': ','.join(swim_list)}


def variants(r, swim_flat):
    e = r['ends']
    c_sw = e + r['changes'] + r['swim_ch'] + r['cross']
    c_flat = e + r['changes'] + swim_flat * r['resid'] + r['cross']
    lmm = r.get('lane_mm') or 0.0
    # the per-swimmer clamp (the judge review's interim term): sum(min(swim_changes_i, 3))
    sl = [int(v) for v in (r.get('swim_list') or '').split(',') if v not in ('', 'None')]
    c_cap3 = e + r['changes'] + sum(min(v, 3) for v in sl) + 3 * (r['resid'] - len(sl)) + r['cross']
    return {'resid': r['resid'],
            'model': r['model'] if r['model'] is not None else float('nan'),
            'c_sw': c_sw,
            'c_flat': c_flat,
            'c_cap3': c_cap3,
            'c_sw_len': c_sw + lmm / VIA_MM,          # Andy's rule: + the braid's planned lane length at VIA_MM
            'c_flat_len': c_flat + lmm / VIA_MM}


def spearman(xs, ys):
    n = len(xs)
    if n < 3:
        return float('nan')

    def ranks(v):
        order = sorted(range(n), key=lambda i: v[i])
        rk = [0.0] * n
        i = 0
        while i < n:
            j = i
            while j + 1 < n and v[order[j + 1]] == v[order[i]]:
                j += 1
            for k in range(i, j + 1):
                rk[order[k]] = (i + j) / 2 + 1
            i = j + 1
        return rk
    rx, ry = ranks(xs), ranks(ys)
    mx, my = sum(rx) / n, sum(ry) / n
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    den = math.sqrt(sum((a - mx) ** 2 for a in rx) * sum((b - my) ** 2 for b in ry))
    return num / den if den else float('nan')


def concordance(vals, routed, pair_min, margin):
    """(pairs, concordant, false accepts): over pairs with |routed_i -
    routed_j| >= pair_min, concordant = the judge orders them as the
    copper did (strictly); false accept = the judge prefers the WORSE
    board by more than `margin`."""
    n = len(vals)
    pairs = conc = false = 0
    for i in range(n):
        for j in range(i + 1, n):
            dr = routed[i] - routed[j]
            if abs(dr) < pair_min or any(math.isnan(v) for v in (vals[i], vals[j])):
                continue
            dv = vals[i] - vals[j]
            pairs += 1
            if dv * dr > 0:
                conc += 1
            elif dv * dr < 0 and abs(dv) > margin:
                false += 1
    return pairs, conc, false


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--roots', default='tmp,tmp/s7,tmp/s8,tmp/s9')
    ap.add_argument('--ks', default='28,35,41,51')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--out', default='tmp/s10/judge_gate.tsv')
    ap.add_argument('--only', default='')
    ap.add_argument('--margin', type=float, default=0.0)
    ap.add_argument('--pair-min', type=int, default=5)
    ap.add_argument('--open-penalty', type=float, default=20.0)
    ap.add_argument('--swim-flat', type=float, default=None,
                    help='the flat swimmer price (default prices.SWIM)')
    ap.add_argument('--main-only', action='store_true',
                    help='drop arms whose braid or planner differed (arc*, rp*, pf*, bs*, ...)')
    ap.add_argument('--dedupe', action='store_true',
                    help='one row per distinct (K, judge inputs, routed) -- the ladder re-runs identical plans many times')
    a = ap.parse_args()
    import prices
    swim_flat = a.swim_flat if a.swim_flat is not None else prices.SWIM
    ks = [int(k) for k in a.ks.split(',') if k]
    only = {t for t in a.only.split(',') if t}

    rows = {}
    if os.path.isfile(a.out):
        with open(a.out, encoding='utf-8') as f:
            hdr = None
            for line in f:
                parts = line.rstrip('\n').split('\t')
                if hdr is None:
                    hdr = parts
                    continue
                r = dict(zip(hdr, parts))
                for k in ('K', 'n', 'resid', 'ends', 'changes', 'swim_ch', 'cross', 'routed', 'open', 'nolane'):
                    r[k] = int(r[k]) if r.get(k) not in (None, '', 'None') else 0
                for k in ('lane_mm', 'routed_mm'):
                    r[k] = float(r[k]) if r.get(k) not in (None, '', 'None') else 0.0
                r['model'] = int(r['model']) if r['model'] not in ('', 'None') else None
                r['pred_flat'] = float(r['pred_flat']) if r['pred_flat'] not in ('', 'None') else None
                rows[(r['dir'], r['tag'], r['K'])] = r

    names_of = {K: coherent(K) for K in ks}
    jobs = []
    for d in a.roots.split(','):
        for p in sorted(glob.glob(os.path.join(d, '*_fo_k*.plan.json'))):
            m = re.match(r'(.+)_fo_k(\d+)\.plan\.json$', os.path.basename(p))
            if not m:
                continue
            tag, K = m.group(1), int(m.group(2))
            if K not in ks or (only and tag not in only):
                continue
            fo = os.path.join(d, f'{tag}_fo_k{K}.kicad_pcb')
            rt = os.path.join(d, f'{tag}_k{K}.kicad_pcb')
            if not (os.path.isfile(fo) and os.path.isfile(rt)) or (d, tag, K) in rows:
                continue
            jobs.append((d, tag, K, names_of[K]))
    print(f'{len(rows)} cached row(s), {len(jobs)} board(s) to grade', flush=True)
    if jobs:
        if a.workers > 1:
            from multiprocessing import Pool
            with Pool(a.workers) as pool:
                results = pool.imap_unordered(grade_one, jobs)
                results = list(_progress(results, len(jobs)))
        else:
            results = list(_progress(map(grade_one, jobs), len(jobs)))
        for r in results:
            if 'error' in r:
                print(f'  {r["dir"]}/{r["tag"]} K{r["K"]}: {r["error"]}', flush=True)
                continue
            rows[(r['dir'], r['tag'], r['K'])] = r
        os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
        with open(a.out, 'w', encoding='utf-8') as f:
            f.write('\t'.join(FIELDS) + '\n')
            for key in sorted(rows):
                r = rows[key]
                f.write('\t'.join(str(r.get(k, '')) for k in FIELDS) + '\n')
        print(f'{len(rows)} row(s) -> {a.out}', flush=True)

    vnames = ['resid', 'model', 'c_sw', 'c_flat', 'c_cap3', 'c_sw_len', 'c_flat_len']
    for K in ks:
        rs = [r for key, r in sorted(rows.items()) if r['K'] == K]
        if a.main_only:
            rs = [r for r in rs if not OTHER.match(r['tag'])]
        if a.dedupe:
            seen, uniq = set(), []
            for r in rs:
                sig = (r['resid'], r['ends'], r['changes'], r['swim_ch'], r['cross'], r['routed'], r['open'])
                if sig not in seen:
                    seen.add(sig)
                    uniq.append(r)
            rs = uniq
        if not rs:
            continue
        print(f'\n=== K{K}: {len(rs)} board(s)' + (' main-only' if a.main_only else '') + (' deduped' if a.dedupe else ''))
        print(f'{"dir/tag":22s} {"resid":>5s} {"model":>5s} {"c_sw":>5s} {"c_flat":>6s} {"c_cap3":>6s} '
              f'{"lane_mm":>7s} {"c_sw_len":>8s} | {"routed":>6s} {"mm":>6s} {"v+mm/7.5":>8s} open')
        for r in rs:
            v = variants(r, swim_flat)
            print(f'{r["dir"] + "/" + r["tag"]:22s} {v["resid"]:5d} {v["model"]:5.0f} {v["c_sw"]:5d} '
                  f'{v["c_flat"]:6.0f} {v["c_cap3"]:6.0f} {r.get("lane_mm", 0):7.0f} {v["c_sw_len"]:8.1f} | {r["routed"]:6d} '
                  f'{r.get("routed_mm", 0):6.0f} {r["routed"] + r.get("routed_mm", 0) / VIA_MM:8.1f} '
                  f'{r["open"]}{" " + r["opens"] if r["opens"] else ""}')
        routed = [r['routed'] for r in rs]
        eff = [r['routed'] + a.open_penalty * r['open'] for r in rs]
        # Andy's rule as the TARGET: routed vias + routed mm / VIA_MM (+ the open penalty)
        rule = [r['routed'] + r.get('routed_mm', 0) / VIA_MM + a.open_penalty * r['open'] for r in rs]
        clean = [i for i, r in enumerate(rs) if r['open'] == 0]
        print(f'  {"judge":10s} {"rho(clean)":>10s} {"rho(eff)":>9s} {"rho(rule)":>9s} | pairs conc  false  (|d routed| >= {a.pair_min}, margin {a.margin:g}; rule = vias + mm/7.5)')
        for vn in vnames:
            vals = [variants(r, swim_flat)[vn] for r in rs]
            rc = spearman([vals[i] for i in clean], [routed[i] for i in clean])
            re_ = spearman(vals, eff)
            rr = spearman(vals, rule)
            p, c, fa = concordance(vals, eff, a.pair_min, a.margin)
            print(f'  {vn:10s} {rc:10.2f} {re_:9.2f} {rr:9.2f} | {p:5d} {c / p if p else float("nan"):5.2f} '
                  f'{fa / p if p else float("nan"):5.2f}')
        lm = [r.get('lane_mm', 0) for r in rs]
        rm = [r.get('routed_mm', 0) for r in rs]
        if any(lm):
            print(f'  length estimate: rho(lane_mm, routed_mm) = {spearman(lm, rm):.2f}; '
                  f'mean lane/routed = {sum(lm) / max(1e-9, sum(rm)):.2f}; nets without a lane: '
                  f'{sum(r.get("nolane", 0) for r in rs)}')


def _progress(it, total):
    done = 0
    for r in it:
        done += 1
        if done % 10 == 0 or done == total:
            print(f'  graded {done}/{total}', flush=True)
        yield r


if __name__ == '__main__':
    main()
