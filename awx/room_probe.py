#!/usr/bin/env python3
"""Does ROOM predict a swimmer's vias where the crossing COUNT does not?

The crossing count is what `SF_SWIM_MODEL` prices and it carries no signal
(corr with routed vias +0.016, sign-unstable per board). The mechanism for
that is known: the braid ABSORBS most predicted crossings without a via --
it routes around them -- and whether it can is a question of local ROOM.
This asks whether room is visible in the plan, BEFORE any term is built on
it, over the same per-swimmer crossing sequences the judge already computes.

usage: room_probe.py FANOUT_BOARD ROUTED_BOARD [FANOUT_BOARD ROUTED_BOARD ...]
"""
import io, json, os, re, sys, contextlib, statistics as st
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb          # noqa: E402
import braid as te                                 # noqa: E402


def sequences(fo_board, plan_path, names, dref='DU1'):
    """Per swimmer, the ORDERED crossings it makes -- (position along the
    lane, the page of the lane it crosses) -- plus every crossing position
    in its corridor, which is what 'room' has to be measured against."""
    plan = json.load(open(plan_path))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        bp = te.plan_braid(fo_board, list(names), dref, plan)
    by_c = {}
    for nm, d in bp.items():
        li, ti = d.get('launch_idx'), d.get('target_idx')
        if li is None or ti is None:
            continue
        by_c.setdefault(d.get('corridor'), []).append((nm, li, ti, d.get('page')))
    out = {}
    for lanes in by_c.values():
        # every crossing position in this corridor, page lanes included:
        # the lane's room is what the NEIGHBOURHOOD is doing, not just it
        allx = []
        for nm, ls, ts, _pg in lanes:
            for om, lj, tj, _pj in lanes:
                if om == nm or (lj < ls) == (tj < ts):
                    continue
                den = (lj - ls) - (tj - ts)
                allx.append(((lj - ls) / den) if den else 0.5)
        for nm, ls, ts, pg in lanes:
            if pg is not None:
                continue
            seq = []
            for om, lj, tj, pj in lanes:
                if om == nm or pj is None or (lj < ls) == (tj < ts):
                    continue
                den = (lj - ls) - (tj - ts)
                seq.append((((lj - ls) / den) if den else 0.5, str(pj)))
            seq.sort()
            out[nm] = (seq, sorted(allx))
    return out


def features(seq, allx):
    """Candidate ROOM statistics for one swimmer."""
    f = {'xing': len(seq)}
    alts = [(a[0], b[0]) for a, b in zip(seq, seq[1:]) if a[1] != b[1]]
    f['chg'] = len(alts)
    gaps = [b - a for a, b in alts]
    f['gap_min'] = min(gaps) if gaps else 1.0
    f['gap_mean'] = st.mean(gaps) if gaps else 1.0
    # a change with no LONGITUDINAL room: the two crossings it sits between
    # are close, so there is nowhere to dive and come back
    for thr in (0.02, 0.05, 0.10):
        f[f'tight{thr}'] = sum(1 for g in gaps if g < thr)
        f[f'loose{thr}'] = sum(1 for g in gaps if g >= thr)
    # LATERAL crowding: how many crossings of ANY lane sit in the window a
    # change would need. Many -> the lane cannot dodge and must pay.
    for w in (0.05, 0.10):
        c = 0
        for a, b in alts:
            mid = 0.5 * (a + b)
            c += sum(1 for t in allx if abs(t - mid) <= w)
        f[f'crowd{w}'] = c
        f[f'crowd{w}_per'] = c / max(len(alts), 1)
    # room-WEIGHTED change count: a change in a crowded window costs full,
    # one with space around it is discounted
    for w in (0.05, 0.10):
        tot = 0.0
        for a, b in alts:
            mid = 0.5 * (a + b)
            n = sum(1 for t in allx if abs(t - mid) <= w)
            tot += n / (n + 2.0)
        f[f'roomw{w}'] = tot
    return f


def corr(a, b):
    ma, mb = st.mean(a), st.mean(b)
    n = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    d = (sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b)) ** .5
    return n / d if d else float('nan')


if __name__ == '__main__':
    import subprocess
    pairs = list(zip(sys.argv[1::2], sys.argv[2::2]))
    ROWS = []
    for fo, routed in pairs:
        K = re.search(r'_k(\d+)', os.path.basename(fo)).group(1)
        nets = subprocess.run(['python3', os.path.join(HERE, 'coherent_nets.py'), K,
                               f'--board={os.path.join(HERE, "fb_t2q_fresh.kicad_pcb")}'],
                              capture_output=True, text=True).stdout.strip().split(',')
        plan = fo.replace('.kicad_pcb', '.plan.json')
        if not os.path.exists(plan):
            print(f'  skip {os.path.basename(fo)}: no plan sidecar'); continue
        try:
            seqs = sequences(fo, plan, nets)
        except Exception as e:
            print(f'  skip {os.path.basename(fo)}: {type(e).__name__}: {e}'); continue
        p = parse_kicad_pcb(routed)
        bn = {n.name.split('/')[-1]: i for i, n in p.nets.items()}
        n_add = 0
        for nm, (seq, allx) in seqs.items():
            if nm not in bn:
                continue
            f = features(seq, allx)
            f['vias'] = sum(1 for v in p.vias if v.net_id == bn[nm])
            f['net'], f['board'] = nm, os.path.basename(routed)
            ROWS.append(f); n_add += 1
        print(f'  {os.path.basename(routed)}: {n_add} swimmer(s)')
    if not ROWS:
        sys.exit('no rows')
    print(f'\n{len(ROWS)} swimmers pooled; vias mean '
          f'{st.mean([r["vias"] for r in ROWS]):.2f} sd {st.pstdev([r["vias"] for r in ROWS]):.2f}\n')
    va = [r['vias'] for r in ROWS]
    keys = [k for k in ROWS[0] if k not in ('vias', 'net', 'board')]
    print(f'{"feature":16} {"corr":>7}')
    for k in sorted(keys, key=lambda k: -abs(corr([r[k] for r in ROWS], va))):
        print(f'  {k:14} {corr([r[k] for r in ROWS], va):+7.3f}')
    json.dump(ROWS, open(os.path.join(HERE, 'tmp', 'room_rows.json'), 'w'), indent=1)
