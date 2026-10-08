#!/usr/bin/env python3
"""whole_movie.py RUNDIR OUT.mp4 -- a film of one whole-route run (#622), from the fanout to the final copper.

RUNDIR is whole_route.py's OUTDIR; the round that routed (rN/seq.kicad_pcb) is filmed. The film walks the stages the
chain ran, in order -- the ends, the frame, the solve, the geometry, the polish, the snap, the audit, the route -- on
the board, and draws the two SOLVERS' own decision spaces beside it:

  THE SOLVE (CP-SAT) as a BRAID under the board. Its x is the route coordinate u, which on the trunk IS distance
  along the board (the canonical frame runs source to destination along +x), so the braid shares the board's x
  axis; its y is each lane's ORDER, north to south; the colour is the layer. The solve's variables are where each
  crossing and each layer change sits along u, and its rules can be seen there: two strands that cross differ in
  colour, a white ring is a via (a colour change). The search is replayed from the plans CP-SAT really found, in
  the order it found them, with the bound that proved the last one. On the board above, the same plan is drawn in
  the references' slots (each column's reference offsets, handed out in the solve's order): the solve knows order
  and layers, not millimetres, and the lanes swap exactly where it put each crossing.

  THE GEOMETRY (the LP) as FORCES on the board. The LP's dual gives every rule a SHADOW PRICE; the rules with a
  price are the ones holding a lane where it is -- a same-layer neighbour at the bar, a via's room, a pad box or
  the board edge, an island's side, a turn limit -- and the price is how hard they push. A cursor column shows one
  column of the LP: its variables are those lanes' offsets, in the braid's order at the same u.

Both solvers are OBSERVED, never re-implemented: the round's root solve and its geometry are run again in-process
under the chain's own environment, CP-SAT with a solution callback and whole_geo under a profile hook that keeps
each LP pass's matrix, row tags and the dual HiGHS already solves. Each re-run must write what the chain wrote, byte
for byte, or the film is refused (--force films it anyway, and says so on screen). The traces are kept in
RUNDIR/rN/movie/.

usage: whole_movie.py RUNDIR OUT.mp4 [--fps 30] [--workers 6] [--stills DIR [--at T1,T2,..]] [--human BOARD]
                     [--speed X] [--force] [--scenes]
The bench is the chain's: BASE (default fb_t2q_pairs.kicad_pcb) and DEST (default DU1) as whole_route.py reads them.
Needs ffmpeg (libx264) for the mp4; --stills alone writes PNGs.
"""
KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import collections
import contextlib
import functools
import io
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PYR = os.path.normpath(os.path.join(HERE, '..', 'py_router'))


# =============================================================================== tracing (runs in a subprocess)
def _chain_env(round_dir, dest):
    """the environment whole_route.py gives a stage of this round"""
    nets = [ln.strip() for ln in open(os.path.join(round_dir, 'nets.lines')) if ln.strip()]
    env = dict(os.environ)
    for k in ('HINT', 'CUTS', 'HIST', 'GEO_FLIPS_FROM', 'SEED_FLIPS', 'SEED_CUTS', 'SEED_HIST', 'SNAP_KEEP'):
        env.pop(k, None)
    env.update(OMP_NUM_THREADS='1', VECLIB_MAXIMUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
               TAUT_MEMO=os.environ.get('TAUT_MEMO', '1'), PROBE_MEMO=os.environ.get('PROBE_MEMO', '1'),
               PLAN_PAGES='1', PLAN_JUDGE='ends', BRAID_PAIRS='1', PLAN_PAIRS='1',
               BRAID_EXACT_PAGES='0', PLAN_PAGES_SIDERS='2', STAGE_CACHE=os.environ.get('STAGE_CACHE', '1'),
               BENCH=os.path.join(round_dir, 'fo.kicad_pcb'), NETS=','.join(nets), DEST=dest)
    return env


def _ix(v):
    return v.Index() if callable(getattr(v, 'Index', None)) else v.index


def _trace_solve(round_dir, out):
    """the round's ROOT solve (its solve.json) run again with every plan CP-SAT finds kept, and the model's own
    reading of the bench (the crossing windows, the face band, the built-in via cuts) from solve()'s locals"""
    sys.path.insert(0, HERE)
    from ortools.sat.python import cp_model
    runs = []
    orig = cp_model.CpSolver.Solve

    class Plans(cp_model.CpSolverSolutionCallback):
        def __init__(self, model, run):
            super().__init__()
            self.vs = [model.GetIntVarFromProtoIndex(i) for i in range(len(model.Proto().variables))]
            self.run = run

        def on_solution_callback(self):
            self.run['plans'].append({'t': self.WallTime(), 'obj': self.ObjectiveValue(),
                                      'bound': self.BestObjectiveBound(), 'x': [self.Value(v) for v in self.vs]})

    def observed(self, model, solution_callback=None):
        run = {'plans': [], 'log': []}
        runs.append(run)
        passed = self.log_callback

        def log(line):
            run['log'].append(line)
            if passed is not None:
                passed(line)
        self.log_callback = log
        return orig(self, model, Plans(model, run))
    cp_model.CpSolver.Solve = observed
    got = {}

    def hook(frame, event, arg):
        if event == 'return' and frame.f_code.co_name == 'solve' and frame.f_code.co_filename.endswith('whole_solve.py'):
            got.update(frame.f_locals)
    import whole_ctx
    import whole_solve
    ctx, _cs = whole_ctx.plan()
    sys.setprofile(hook)
    try:
        J = whole_solve.solve(ctx, os.environ['DEST'])
    finally:
        sys.setprofile(None)
    if J is None:
        raise SystemExit('whole_movie: the root solve proved no plan')
    J = json.loads(json.dumps(J))
    ref = json.load(open(os.path.join(round_dir, 'solve.json')))
    L = got
    Gs = whole_solve.G

    def decode(x):
        cross = {f'{a}|{b}': x[_ix(v)] * Gs for (a, b), v in L['t'].items()}
        changes = {n: [x[_ix(cv)] * Gs for cv, av in zip(*L['chg'][n]) if x[_ix(av)]] for n in L['M']}
        return cross, changes, int(sum(x[_ix(L['over'][n])] for n in L['M']))
    out_runs = []
    for i, r in enumerate(runs):
        plans = []
        for p in r['plans']:
            cross, changes, over = decode(p['x'])
            plans.append({'t': p['t'], 'obj': p['obj'], 'bound': p['bound'], 'cross': cross, 'changes': changes,
                          'over': over, 'vias': sum(len(v) for v in changes.values())})
        out_runs.append({'workers': 'bound-raising' if i == 0 else 'fallback', 'plans': plans,
                         'log': [ln for ln in r['log'] if ln.startswith('#')]})
    fl = lambda d: {k: float(v) for k, v in d.items()}
    res = {'identical': json.dumps(J, sort_keys=True) == json.dumps(ref, sort_keys=True),
           'final': J, 'runs': out_runs, 'G': Gs, 'W_V': whole_solve.W_V, 'W_OVER': L['W_OVER'],
           'KMAX': whole_solve.KMAX, 'M': list(L['M']), 'launch': list(L['Ln']), 'final_order': list(L['Fn']),
           'windows': {f'{a}|{b}': [float(v[0]), float(v[1])] for (a, b), v in L['win'].items()},
           'entry': fl(L['entry']), 'end': fl(L['end']), 'tend': fl(L['tend']), 'tl': L['tl'], 'dl': L['dl'],
           'branch': dict(L['bname']), 'Hn': fl(L['Hn']), 'BAND': float(L['BAND']), 'S_FACE': float(L['S_FACE']),
           'vcuts': [{'lane': c['lane'], 'u': float(c['u']), 'w': float(c['w'])} for c in L['VCUTS']],
           'pairs': sorted(n for n in (L['prs'] or {}) if n in L['M']), 'xo': sorted(L['XO']), 'SV': L['SV'],
           'triples': int(L['nt'])}
    json.dump(res, open(out, 'w'))
    print(f'whole_movie: root solve traced -- {sum(len(r["plans"]) for r in out_runs)} plan(s), identical to the '
          f'chain: {res["identical"]}')


def _trace_geo(round_dir, solve_path, flips, gref, out):
    """the round's geometry (whole_geo on its solve and flips) run again under a profile hook: each LP pass's
    matrix, row tags, primal and DUAL (every rule's shadow price), the static sides, and the frame it built"""
    sys.path.insert(0, HERE)
    import filecmp
    import runpy
    import numpy as np
    passes, sides, duals = [], [], []

    def hook(frame, event, arg):
        co = frame.f_code
        if not co.co_filename.endswith('whole_geo.py'):
            return
        if event == 'call' and co.co_name == 'build_and_solve':
            g = frame.f_globals
            if not getattr(g['linprog'], '_observed', False):
                lp = g['linprog']

                def observed(*a, **k):
                    r = lp(*a, **k)
                    duals.append(np.array(r.x) if r.status == 0 else None)
                    return r
                observed._observed = True
                g['linprog'] = observed
        elif event == 'return' and co.co_name == 'build_and_solve' and arg is not None:
            L = frame.f_locals
            passes.append(dict(A=L['Aub'], b=np.asarray(L['rhs'], float), tags=list(L['tags']), var=dict(L['var']),
                               x=np.asarray(L['res'].x), sol=arg, dual=duals[-1] if duals else None))
        elif event == 'return' and co.co_name == 'static_sides':
            sides.append(arg)
    GEO = os.path.join(HERE, 'whole_geo.py')
    tmp = out + '.g.json'
    sys.argv = [GEO, solve_path, tmp]
    if flips:
        os.environ['GEO_FLIPS_FROM'] = flips
    buf = io.StringIO()
    sys.setprofile(hook)
    try:
        with contextlib.redirect_stdout(buf):
            G = runpy.run_path(GEO, run_name='__main__')
    finally:
        sys.setprofile(None)
    identical = filecmp.cmp(tmp, gref, shallow=False)
    FR, PIECE, Fr, Gc = G['FR'], G['PIECE'], G['Fr'], G['G']
    svx = G['spine_xy_vec']
    cols = {}

    def col(f, k):
        if (f, k) not in cols:
            s = k * Gc
            X, Y = svx(FR[f]['sp'], s, np.array([0.0, 1.0]))
            cols[(f, k)] = {'f': f, 'k': k, 'u': float(FR[f]['u'](s)), 'b': [float(X[0]), float(Y[0])],
                            'n': [float(X[1] - X[0]), float(Y[1] - Y[0])]}
        return cols[(f, k)]
    route_key = lambda fk: (0 if fk[0] == 'T' else 1, fk[1])
    out_passes = []
    for P in passes:
        o = P['sol']['o']
        lanes = collections.defaultdict(list)
        for (f, n, k), v in o.items():
            col(f, k)
            lanes[n].append([f, k, float(v)])
        for n in lanes:
            lanes[n].sort(key=lambda e: route_key(e))
        A, Ar, b, x, y = P['A'].tocsc(), P['A'].tocsr(), P['b'], P['x'], P['dual']
        slack = b - Ar @ x
        best = {}
        for e, tg in P['tags']:
            rr = A.indices[A.indptr[e]:A.indptr[e + 1]]
            if not len(rr):
                continue
            r = int(rr[0])
            price = float(y[r]) if y is not None else 0.0
            key = repr(tg)
            cur = best.get(key)
            if cur is None or price > cur['price']:
                best[key] = {'tag': tg, 'r': r, 'price': price, 'slack': float(slack[r]),
                             'paid': max(float(x[e]), cur['paid'] if cur else 0.0)}
            else:
                cur['paid'] = max(cur['paid'], float(x[e]))

        def lane_pt(f, n, k):
            v = o.get((f, n, k))
            if v is None:
                return None
            c = col(f, k)
            return [c['b'][0] + v * c['n'][0], c['b'][1] + v * c['n'][1]]
        rows = []
        for R in best.values():
            if R['price'] < 1e-7 and R['paid'] < 1e-4:
                continue
            tg = R['tag']
            kind, f, k = tg[0], tg[1], int(tg[2])
            what, wall = '', 0
            if kind in ('pitch', 'via', 'viavia'):
                lanes_ = [tg[3], tg[4]]
                pts = [lane_pt(f, tg[3], k), lane_pt(f, tg[4], k)]
            elif kind in ('bound', 'static'):
                # a bound on one offset: the lane stands AT it when it binds -- the wall is on the side the
                # coefficient says (+1: an upper bound, the wall at larger offsets)
                n = tg[3]
                j = P['var'].get((f, n, k))
                coef = float(Ar[R['r'], j]) if j is not None else 0.0
                lanes_ = [n]
                pts = [lane_pt(f, n, k)]
                what = str(tg[4])
                wall = (1 if coef > 0 else -1) if coef else 0
            else:                                   # slope, turn, pturn, pdive, approach: a lane's own shape
                lanes_ = [tg[3]]
                pts = [lane_pt(f, tg[3], k)]
            if not pts or pts[0] is None:
                continue
            rows.append({'kind': kind, 'f': f, 'k': k, 'lanes': lanes_, 'price': R['price'], 'paid': R['paid'],
                         'slack': R['slack'], 'pts': [p for p in pts if p is not None], 'what': what,
                         'wall': wall, 'nv': col(f, k)['n']})
        out_passes.append({'lanes': lanes, 'rows': rows, 'nvar': len(P['var']), 'nrow': int(len(b)),
                           'paid': {k: len(v) for k, v in P['sol']['paid'].items()}})
    # each lane's reference offset (the LP's bound picks the free interval nearest it) at every column either pass
    # has -- the second pass starts a ring piece where its trunk ended (whole_geo.reanchor)
    refs = {}
    for P in passes:
        for (f, n, k) in P['var']:
            refs.setdefault(n, {})[(f, k)] = float(PIECE[(f, n)]['ref'](k * Gc))
    refs = {n: sorted([[f, k, o] for (f, k), o in v.items()], key=route_key) for n, v in refs.items()}
    side_of = {}
    for (f, n, k, lo_, hi_, side, what) in (sides[-1] if sides else []):
        e = side_of.setdefault((f, n, what), {'f': f, 'lane': n, 'island': what, 'side': side, 'k0': k, 'k1': k})
        e['k0'], e['k1'] = min(e['k0'], k), max(e['k1'], k)
    ctx = G['ctx']
    Mf = getattr(ctx, 'M', None)
    aff = None
    if Mf is not None:
        o0, ex, ey = Mf(0.0, 0.0), Mf(1.0, 0.0), Mf(0.0, 1.0)
        aff = [list(map(float, o0)), [float(ex[0] - o0[0]), float(ex[1] - o0[1])], [float(ey[0] - o0[0]), float(ey[1] - o0[1])]]
    P2 = lambda p: [float(p[0]), float(p[1])]
    res = {'identical': identical, 'flips': flips, 'G': Gc, 'cols': list(cols.values()), 'passes': out_passes,
           'refs': refs, 'sides': list(side_of.values()),
           'islands': [{'box': [float(s[0]), float(s[1]), float(s[2]), float(s[3])], 'layers': sorted(s[4]),
                        'label': s[5], 'own': s[6] if len(s) > 6 else None} for s in G['STATIC']],
           'M': list(G['M']), 'cls': dict(G['cls']), 'pairs': {n: list(v) for n, v in G['prs'].items() if n in G['M']},
           'tooth': {n: P2(v[0]) for n, v in G['TERM'].items()}, 'berth': {n: P2(v[1]) for n, v in G['TERM'].items()},
           'start': {n: P2(Fr.start[n]) for n in G['M']}, 'land': {n: P2(Fr.land[n]) for n in G['M']},
           'spine': [P2(p) for p in Fr.spine.P], 'rings': {k: [P2(p) for p in r.P] for k, r in Fr.rings.items()},
           'ring_u': {f: [float(G['HK'][f]), float(G['S0C'][f])] for f in G['S0C']},
           'SB': list(map(float, Fr.SB)), 'DB': list(map(float, Fr.DB)), 'H0': float(Fr.H0),
           'Hk': {k: float(v) for k, v in Fr.Hk.items()}, 'src': Fr.src, 'dst': Fr.dst,
           'chi': int(getattr(ctx, 'chi', 1) or 1), 'aff': aff,
           'rules': {'P_MIN': float(G['P_MIN']), 'P_COMF': float(G['P_COMF']), 'TW': float(G['TW']),
                     'CL': float(G['CL']), 'VIA_R': float(G['VIA_R']), 'PP': float(G['PP']),
                     'grid': float(ctx.cfg.grid_step), 'via': float(ctx.cfg.via_size)},
           'log': [ln for ln in buf.getvalue().splitlines() if 'joint LP' in ln or ln.startswith(('pass ', 'cuts', '  PAID'))]}
    json.dump(res, open(out, 'w'))
    print(f'whole_movie: geometry traced -- {len(passes)} LP pass(es), identical to the chain: {identical}')


# =============================================================================== the run's files
def _read(path):
    return open(path).read() if os.path.isfile(path) else ''


def _parse(path):
    sys.path.insert(0, PYR)
    with contextlib.redirect_stdout(io.StringIO()):
        from kicad_parser import parse_kicad_pcb
        return parse_kicad_pcb(path)


def loop_rounds(round_dir):
    """the loop's rounds as whole_route.py ran them, replayed from its log: each round's solve and the side flips its
    geometry was given (GEO_FLIPS_FROM), and the round whose plan passed"""
    loop = os.path.join(round_dir, 'loop')
    rounds, flips, cur, passed = {}, [], None, None
    for line in _read(os.path.join(round_dir, 'loop.log')).splitlines():
        m = re.match(r'=== round (\d+): geometry of (\S+)', line)
        if m:
            cur = int(m.group(1))
            s = m.group(2)
            rounds[cur] = {'i': cur, 'solve': os.path.join(round_dir, s) if s == 'solve.json' else os.path.join(loop, s),
                           'flips': list(flips), 'lines': [line]}
            continue
        if cur is not None:
            rounds[cur]['lines'].append(line)
        m = re.match(r'=== round (\d+): (\d+) new side flip', line)
        if m:
            flips = [os.path.join(loop, f'p{m.group(1)}.json')]
        if re.match(r'\s+pairs held: \d+ new side flip', line) and cur is not None:
            qk = os.path.join(loop, f'qk{cur}.json')
            flips.append(qk if os.path.isfile(qk) else os.path.join(loop, f'q{cur}.json'))
        if line.startswith('=== the plan passes'):
            passed = cur
    return rounds, passed


class Story:
    """everything the film shows, read from the run and the traces"""


def load_story(a):
    S = Story()
    rdir = os.path.abspath(a.rundir)
    rs = sorted(int(m.group(1)) for d in os.listdir(rdir) for m in [re.match(r'r(\d+)$', d)] if m)
    routed = [r for r in rs if os.path.isfile(os.path.join(rdir, f'r{r}', 'seq.kicad_pcb'))]
    if not routed:
        raise SystemExit(f'whole_movie: no round of {rdir} routed (rN/seq.kicad_pcb) -- the film is of a run that routed')
    S.rn = routed[-1]
    S.nrounds = len(rs)
    rd = os.path.join(rdir, f'r{S.rn}')
    S.rd, S.loop = rd, os.path.join(rd, 'loop')
    S.dest = os.environ.get('DEST', 'DU1')
    S.base_path = os.environ.get('BASE', 'fb_t2q_pairs.kicad_pcb')
    if not os.path.isabs(S.base_path):
        S.base_path = os.path.join(HERE, S.base_path)
    if S.rn > 1:        # the round's own input: the previous round's source board (whole_route.py's BASE for it)
        sb = re.findall(r'source board: ([^,]+)', _read(os.path.join(rdir, f'r{S.rn - 1}', 'fo.log')))
        if sb and os.path.isfile(os.path.join(rdir, f'r{S.rn - 1}', sb[-1].strip())):
            S.base_path = os.path.join(rdir, f'r{S.rn - 1}', sb[-1].strip())
    rounds, passed = loop_rounds(rd)
    if passed is None:
        raise SystemExit(f'whole_movie: {rd}/loop.log names no round whose plan passed')
    S.rounds, S.lr = rounds, passed
    # ---- the traces (observed re-runs), cached beside the run
    mv = os.path.join(rd, 'movie')
    os.makedirs(mv, exist_ok=True)
    env = _chain_env(rd, S.dest)
    ts, tg = os.path.join(mv, 'trace_solve.json'), os.path.join(mv, f'trace_geo{passed}.json')
    R = rounds[passed]
    if not os.path.isfile(ts):
        subprocess.run([sys.executable, os.path.abspath(__file__), '--_trace', 'solve', rd, ts], env=env, check=True)
    if not os.path.isfile(tg):
        subprocess.run([sys.executable, os.path.abspath(__file__), '--_trace', 'geo', rd, R['solve'], ','.join(R['flips']),
                        os.path.join(S.loop, f'g{passed}.json'), tg], env=env, check=True)
    S.T = json.load(open(ts))
    S.Gt = json.load(open(tg))
    S.identical = S.T['identical'] and S.Gt['identical']
    if not S.identical and not a.force:
        raise SystemExit(f'whole_movie: a re-run did not reproduce the chain (solve {S.T["identical"]}, geometry '
                         f'{S.Gt["identical"]}) -- the film would show another run; --force films it anyway')
    # ---- the stages' outputs (the passing round)
    i = passed
    L = S.loop
    ld = lambda f: json.load(open(os.path.join(L, f))) if os.path.isfile(os.path.join(L, f)) else None
    S.geo, S.pol, S.snapd = ld(f'g{i}.json'), ld(f'p{i}.json'), ld('plan.json')
    S.qfile = f'q{i}.json'
    if os.path.isfile(os.path.join(L, f'qk{i}.json')):
        g = subprocess.run([sys.executable, os.path.join(HERE, 'whole_gate.py'), os.path.join(L, f'qk{i}.json'),
                            os.path.join(L, f'qk{i}.audit')], cwd=HERE, env=env, capture_output=True)
        if g.returncode == 0:
            S.qfile = f'qk{i}.json'
    S.pairsd = ld(f'pairs{i}k.json' if S.qfile.startswith('qk') else f'pairs{i}.json')
    S.held = ld(S.qfile)
    S.solve_round = json.load(open(R['solve']))            # the solve the passing round's geometry was laid on
    S.failed = [rounds[r] for r in sorted(rounds) if r < passed]
    # ---- the logs' numbers
    fo = _read(os.path.join(rd, 'fo.log'))
    S.ends = [(m.group(1), int(m.group(3)), int(m.group(4))) for m in
              re.finditer(r'whole ends: (on the teeth as laid|teeth moved \((\d+)\)): (\d+) vias predicted.*?(\d+) crossings', fo)]
    S.gate = {k: v for k, v in re.findall(r'^\s+(smooth|pairs held|snapped): (.*)$', _read(os.path.join(rd, 'loop.log')), re.M)
              if not v.startswith('LINT')}
    S.lint = re.findall(r'^\s+snapped: (LINT.*)$', _read(os.path.join(rd, 'loop.log')), re.M)
    rl = _read(os.path.join(rd, 'route_seq.log'))
    S.route_lines = [ln for ln in rl.splitlines() if re.match(r'^\S+\s+(pair|page)\s+IN BAND', ln)]
    S.route_order = [ln.split()[0] for ln in S.route_lines]
    S.route_summary = next((ln for ln in rl.splitlines() if ln.startswith('SUMMARY')), '')
    S.connected = 'ALL NETS FULLY CONNECTED' in _read(os.path.join(rd, 'conn.log'))
    S.drc_clean = 'NO DRC VIOLATIONS' in _read(os.path.join(rd, 'drc.log'))
    # ---- the lanes and their nets
    Gt = S.Gt
    S.M = list(Gt['M'])
    S.pairs = Gt['pairs']
    S.legs = {n: (S.pairs[n] if n in S.pairs else [n]) for n in S.M}
    S.nets = sorted({leg for n in S.M for leg in S.legs[n]})
    S.lane_of = {leg: n for n in S.M for leg in S.legs[n]}
    S.src, S.dst = Gt['src'], Gt['dst']
    # ---- the boards
    S.base = _parse(S.base_path)
    S.fo = _parse(os.path.join(rd, 'fo.kicad_pcb'))
    S.seq = _parse(os.path.join(rd, 'seq.kicad_pcb'))
    S.human = None
    hp = a.human or os.path.join(HERE, 'fb_t2q_human.kicad_pcb')
    if hp and os.path.isfile(hp):
        h = _parse(hp)
        same = all(r in h.footprints and abs(h.footprints[r].x - S.fo.footprints[r].x) < 1e-3 and
                   abs(h.footprints[r].y - S.fo.footprints[r].y) < 1e-3 for r in (S.src, S.dst))
        S.human = h if same else None
        if not same:
            print(f'whole_movie: {hp} does not place {S.src} and {S.dst} where the bench does -- no human comparison')
    return S


def copper(pcb, names):
    """(segments, vias) of the nets `names` (short names): segments (x0, y0, x1, y1, width, layer, net), vias
    (x, y, size, net)"""
    ids = {i: n.name.split('/')[-1] for i, n in pcb.nets.items() if n.name.split('/')[-1] in names}
    segs = [(s.start_x, s.start_y, s.end_x, s.end_y, s.width, s.layer, ids[s.net_id]) for s in pcb.segments if s.net_id in ids]
    vias = [(v.x, v.y, v.size, ids[v.net_id]) for v in pcb.vias if v.net_id in ids]
    return segs, vias


def _skey(s):
    a, b = (round(s[0] * 1000), round(s[1] * 1000)), (round(s[2] * 1000), round(s[3] * 1000))
    return (min(a, b), max(a, b), s[5], s[6])


def minus(A, B, key):
    """the items of A not in B, as multisets under key"""
    c = collections.Counter(key(x) for x in B)
    out = []
    for x in A:
        k = key(x)
        if c[k]:
            c[k] -= 1
        else:
            out.append(x)
    return out


def copper_len(segs):
    return sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)


# =============================================================================== geometry of the drawing
import numpy as np            # noqa: E402


class Mapper:
    """the plan's frame to the board's: identity, or (a pair chirality of -1) braid.setup's turn back (ctx.M, the
    layers swapped)"""

    def __init__(self, Gt):
        self.aff = Gt.get('aff') if Gt.get('chi', 1) < 0 else None

    def pt(self, x, y):
        if not self.aff:
            return (x, y)
        o, ex, ey = self.aff
        return (o[0] + x * ex[0] + y * ey[0], o[1] + x * ex[1] + y * ey[1])

    def vec(self, dx, dy):
        if not self.aff:
            return (dx, dy)
        _o, ex, ey = self.aff
        return (dx * ex[0] + dy * ey[0], dx * ex[1] + dy * ey[1])

    def bit(self, b):
        return (1 - b) if self.aff else b


def resample(pts, bits, n):
    """a polyline (its per-segment layer bits) resampled to n points by arc length: (n x 2 points, n bits)"""
    P = np.asarray(pts, float)
    if len(P) < 2:
        P = np.vstack([P, P])
        bits = [bits[0] if len(bits) else 0]
    seg = np.hypot(*(P[1:] - P[:-1]).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.linspace(0.0, cum[-1], n)
    i = np.clip(np.searchsorted(cum, t, side='right') - 1, 0, len(seg) - 1)
    f = np.where(seg[i] > 1e-12, (t - cum[i]) / np.where(seg[i] > 1e-12, seg[i], 1.0), 0.0)
    return P[i] + (P[i + 1] - P[i]) * f[:, None], np.asarray(bits, int)[i]


def plen(pts):
    P = np.asarray(pts, float)
    return float(np.hypot(*(P[1:] - P[:-1]).T).sum()) if len(P) > 1 else 0.0


def along(pts, us, u):
    """the point at route coordinate u of a polyline whose vertices carry route coordinates us"""
    for j in range(len(us) - 1):
        if us[j] <= u <= us[j + 1] and us[j + 1] > us[j]:
            f = (u - us[j]) / (us[j + 1] - us[j])
            return (pts[j][0] + (pts[j + 1][0] - pts[j][0]) * f, pts[j][1] + (pts[j + 1][1] - pts[j][1]) * f)
    return tuple(pts[0]) if u < us[0] else tuple(pts[-1])


def simplify(P, eps):
    """a polyline's points with the ones within eps of the chord between their neighbours dropped (Douglas-Peucker)"""
    P = np.asarray(P, float)
    if len(P) < 3:
        return P
    keep = np.zeros(len(P), bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(P) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = P[i], P[j]
        ab = b - a
        L = math.hypot(*ab)
        seg = P[i + 1:j] - a
        d = np.abs(seg[:, 0] * ab[1] - seg[:, 1] * ab[0]) / L if L > 1e-12 else np.hypot(seg[:, 0], seg[:, 1])
        k = int(np.argmax(d))
        if d[k] > eps:
            keep[i + 1 + k] = True
            stack += [(i, i + 1 + k), (i + 1 + k, j)]
    return P[keep]


def nearest(P, p):
    """the point of the (dense) polyline P nearest p"""
    P = np.asarray(P, float)
    return tuple(P[int(np.argmin(((P - np.asarray(p)) ** 2).sum(axis=1)))])


def force_marks(film, rows, st, vs):
    """each priced rule of an LP pass as a mark on the lanes as they are drawn: (row, kind, points). A rule between
    two lanes is drawn ACROSS, lane to lane: the LP's row runs along its column, which a steep lane crosses at a
    slant, so the offsets it separates can stand several bars apart along the column"""
    mp = film.mp
    vias = collections.defaultdict(list)
    for v in vs:
        vias[v[0]].append((v[1], v[2]))
    out = []
    for r in rows:
        pts = [mp.pt(*q) for q in r['pts']]
        ln = r['lanes']
        if r['kind'] in ('pitch', 'via', 'viavia') and len(ln) == 2 and ln[0] in st and ln[1] in st:
            if r['kind'] == 'pitch':
                p0 = nearest(st[ln[0]][0], pts[0])
            else:
                p0 = min(vias[ln[0]], key=lambda v: (v[0] - pts[0][0]) ** 2 + (v[1] - pts[0][1]) ** 2) if vias[ln[0]] else pts[0]
            if r['kind'] == 'viavia' and vias[ln[1]]:
                p1 = min(vias[ln[1]], key=lambda v: (v[0] - p0[0]) ** 2 + (v[1] - p0[1]) ** 2)
            else:
                p1 = nearest(st[ln[1]][0], p0)
            out.append((r, 'strut', [p0, p1]))
        elif r['wall']:
            out.append((r, 'wall', [nearest(st[ln[0]][0], pts[0]) if ln[0] in st else pts[0]]))
        else:
            out.append((r, 'dot', [nearest(st[ln[0]][0], pts[0]) if ln[0] in st else pts[0]]))
    return out


def ease(t):
    t = min(1.0, max(0.0, t))
    return t * t * (3 - 2 * t)


def span(t, a, b):
    return min(1.0, max(0.0, (t - a) / (b - a))) if b > a else float(t >= a)


def lerp(a, b, t):
    return a + (b - a) * t


def mix(c1, c2, t):
    return tuple(int(round(lerp(x, y, t))) for x, y in zip(c1, c2))


class Plan:
    """one CP-SAT plan: crossings (frozenset -> u), changes (lane -> [u]), and its braid"""

    def __init__(self, cross, changes, S):
        # (a solve file's crossing is {'u', 'ring'}, a traced plan's its u alone)
        self.cross = {frozenset(k.split('|')): float(v['u'] if isinstance(v, dict) else v) for k, v in cross.items()}
        self.changes = {n: sorted(map(float, v)) for n, v in changes.items()}
        self.S = S

    def below(self, a, b, u):
        li = self.S.li
        k = frozenset((a, b))
        return (li[a] < li[b]) ^ (k in self.cross and self.cross[k] <= u + 1e-9)

    def layer(self, n, u):
        return self.S.tl[n] ^ (sum(1 for c in self.changes.get(n, ()) if c < u) & 1)


def lerp_plan(A, B, t):
    """the braid between two plans: every crossing and change at its interpolated u (a change the other plan has not
    kept is dropped at the half)"""
    P = Plan({}, {}, A.S)
    P.cross = {k: lerp(A.cross[k], B.cross.get(k, A.cross[k]), t) for k in A.cross}
    for n in set(A.changes) | set(B.changes):
        ca, cb = A.changes.get(n, []), B.changes.get(n, [])
        P.changes[n] = [lerp(x, y, t) for x, y in zip(ca, cb)] if len(ca) == len(cb) else (ca if t < 0.5 else cb)
    return P


# =============================================================================== colours and fonts
BG = (10, 11, 13)
PANEL = (20, 23, 28)
BODY = (26, 34, 28)
TEXT = (228, 232, 238)
DIM = (140, 148, 160)
ACCENT = (255, 210, 90)
F_COL = (240, 84, 66)
B_COL = (70, 156, 250)
LAYC = (F_COL, B_COL)
VIA_COL = (245, 245, 245)
PAIR_COL = (250, 214, 64)
FORCE = {'pitch': (255, 90, 210), 'via': (90, 230, 255), 'viavia': (90, 230, 255), 'bound': (255, 160, 40),
         'static': (255, 160, 40), 'turn': (150, 245, 120), 'pturn': (150, 245, 120), 'pdive': (150, 245, 120),
         'slope': (150, 245, 120), 'approach': (150, 245, 120)}
_FONTS = {}


def font(px, bold=False, mono=False):
    from PIL import ImageFont
    key = (px, bold, mono)
    if key not in _FONTS:
        paths = (['/System/Library/Fonts/Menlo.ttc', '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'] if mono else
                 ['/System/Library/Fonts/Supplemental/Arial Bold.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf']
                 if bold else ['/System/Library/Fonts/Supplemental/Arial.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'])
        f = None
        for p in paths:
            if os.path.isfile(p):
                f = ImageFont.truetype(p, px)
                break
        _FONTS[key] = f or ImageFont.load_default(size=px)
    return _FONTS[key]


# =============================================================================== the film's frame
W, H, SS = 1920, 1080, 2
HEAD = (20, 8, W - 20, 58)
CAP = (20, 62, W - 20, 122)
BOARD = (20, 126, W - 20, 700)
BRAID = (20, 750, W - 20, 1006)
FOOT = (20, 1014, W - 20, 1070)
STAGES = ['ends', 'frame', 'solve', 'geometry', 'polish', 'snap', 'audit', 'route']


class Film:
    """the story prepared for drawing: the board's camera, the braid's axes, every state of every lane"""

    def __init__(self, S, speed=1.0):
        self.S = S
        Gt, T = S.Gt, S.T
        self.mp = Mapper(Gt)
        mp = self.mp
        self.li = {n: i for i, n in enumerate(T['launch'])}
        self.fi = {n: i for i, n in enumerate(T['final_order'])}
        self.tl = {n: mp.bit(int(v)) for n, v in T['tl'].items()}     # (the layer bits of the board's own frame)
        self.dl = {n: mp.bit(int(v)) for n, v in T['dl'].items()}
        self.M = S.M
        self.N = len(self.M)
        self.rules = Gt['rules']
        # ---- plans: CP-SAT's, in the order it found them; the one the passing round was laid on
        self.plans = [Plan(p['cross'], p['changes'], self) for r in T['runs'] for p in r['plans']]
        self.plan_meta = [dict(p, workers=r['workers']) for r in T['runs'] for p in r['plans']]
        self.root = Plan(T['final']['cross'], T['final']['changes'], self)
        self.final = Plan(S.solve_round['cross'], S.solve_round['changes'], self)
        # ---- columns (the LP's own), their route coordinate and board point / normal
        self.cols = {}
        for c in Gt['cols']:
            b = mp.pt(*c['b'])
            nv = mp.vec(*c['n'])
            self.cols[(c['f'], c['k'])] = (c['u'], b, nv)
        self.ends = {n: (mp.pt(*Gt['tooth'][n]), mp.pt(*Gt['start'][n]), mp.pt(*Gt['land'][n]), mp.pt(*Gt['berth'][n]))
                     for n in self.M}
        self._camera()
        self._states()
        self._copper()

    # -------------------------------------------------------------- camera
    def _camera(self):
        S, Gt, mp = self.S, self.S.Gt, self.mp
        pts = []
        for ref in (S.src, S.dst):
            fp = S.fo.footprints[ref]
            pts += [(p.global_x, p.global_y) for p in fp.pads]
        for st in (S.snapd, S.geo):
            for n, v in st['lanes'].items():
                pts += [mp.pt(*p) for p in v['xy']]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        # the braid's u, from the first entry to the last end, on the trunk's line: the board shows it all
        T = S.T
        self.u0 = min(T['entry'].values()) - 0.6
        self.u1 = max(T['end'].values()) + 0.6
        sp = [mp.pt(*p) for p in Gt['spine']]
        d = np.subtract(sp[-1], sp[0])
        d = d / np.hypot(*d)
        self.sp0, self.spd = np.array(sp[0], float), d
        for u in (self.u0, self.u1):
            q = self.sp0 + d * u
            xs.append(q[0])
        m = 0.7
        self.view = (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)
        bw, bh = BOARD[2] - BOARD[0], BOARD[3] - BOARD[1]
        wx, wy = self.view[2] - self.view[0], self.view[3] - self.view[1]
        self.sc = min(bw / wx, bh / wy)                       # px per mm (1x)
        self.ox = BOARD[0] + (bw - wx * self.sc) / 2 - self.view[0] * self.sc
        self.oy = BOARD[1] + (bh - wy * self.sc) / 2 - self.view[1] * self.sc
        # the braid: x of u where the trunk's point at u lands on the board; y of a rank
        # the board's x of u (the trunk's point at u)
        self.board_x = lambda u: self.X(*(self.sp0 + self.spd * u))[0] / SS
        # ...and the braid's: piecewise, the TRUNK between the arrays magnified -- every crossing and change is
        # there, a few millimetres of it -- the run-in along the source's face and the rings compressed; the funnel
        # between the two panels shows which stretch is which
        T = S.T
        h1 = max(T['Hn'].values()) if T['Hn'] else S.Gt['H0']
        self.ku = [self.u0, T['S_FACE'], h1 + 0.4, self.u1]
        xl, xr = BRAID[0] + 120, BRAID[2] - 30
        fr = np.cumsum([0.0, 0.16, 0.54, 0.30])
        self.kx = [xl + (xr - xl) * f for f in fr]
        self.bx = lambda u: float(np.interp(u, self.ku, self.kx))
        self.mag = (self.kx[2] - self.kx[1]) / max(1e-9, self.board_x(self.ku[2]) - self.board_x(self.ku[1]))
        self.down = S.Gt.get('chi', 1) >= 0
        top, bot = BRAID[1] + 22, BRAID[3] - 14
        self.rank_y = lambda r: (top + r * (bot - top) / max(1, self.N - 1)) if self.down else (bot - r * (bot - top) / max(1, self.N - 1))
        self.rstep = (bot - top) / max(1, self.N - 1)

    def X(self, x, y):
        """a board point in supersampled canvas pixels"""
        return ((self.ox + x * self.sc) * SS, (self.oy + y * self.sc) * SS)

    # -------------------------------------------------------------- lane states
    def _column_lane(self, n, keys, o_of, plan):
        """lane n through its columns keys [(f, k)], at offsets o_of(f, k): (points, per-segment bits, route u per
        point) -- from its tooth (and a pair's start past its end run) to its landing and berth"""
        tooth, start, land, berth = self.ends[n]
        pts, us = [], []
        for (f, k) in keys:
            u, b, nv = self.cols[(f, k)]
            o = o_of(f, k)
            pts.append((b[0] + o * nv[0], b[1] + o * nv[1]))
            us.append(u)
        head = [tooth] + ([start] if start != tooth else [])
        tail = [land] + ([berth] if berth != land else [])
        P = head + pts + tail
        U = [us[0]] * len(head) + us + [us[-1]] * len(tail)
        bits = [plan.layer(n, (U[j] + U[j + 1]) / 2 if U[j + 1] > U[j] else (U[j] + (1e-6 if j < len(head) else -1e-6)))
                for j in range(len(P) - 1)]
        return P, bits, U

    def slots(self, plan):
        """a plan drawn in the references' slots: at every column the present lanes' reference offsets, sorted, handed
        out in the plan's order there -- the solve's order and layers, no geometry"""
        Gt = self.S.Gt
        ref = {n: {(f, k): o for f, k, o in Gt['refs'][n]} for n in Gt['refs']}
        keys = {n: [(f, k) for f, k, _o in Gt['passes'][0]['lanes'][n]] for n in self.M}
        at = collections.defaultdict(list)
        for n in self.M:
            for fk in keys[n]:
                at[fk].append(n)
        slot = {}
        for fk, ns in at.items():
            u = self.cols[fk][0]
            od = sorted(ns, key=functools.cmp_to_key(lambda a, b: -1 if plan.below(a, b, u) else 1))
            # (the LP's own convention, every frame: a lane earlier in the order stands at the smaller offset --
            # whole_geo's pitch rows hold o_a + bar <= o_b for a before b)
            offs = sorted(ref[n][fk] for n in ns)
            for n, o in zip(od, offs):
                slot[(n,) + fk] = o
        return {n: self._column_lane(n, keys[n], lambda f, k, n=n: slot[(n, f, k)], plan) for n in self.M}

    def pass_lanes(self, p, plan):
        L = self.S.Gt['passes'][p]['lanes']
        out = {}
        for n in self.M:
            o = {(f, k): v for f, k, v in L[n]}
            out[n] = self._column_lane(n, [(f, k) for f, k, _v in L[n]], lambda f, k, o=o: o[(f, k)], plan)
        return out

    def json_lanes(self, J):
        """a plan file's lanes (geometry, polish, snap): (points, bits, None)"""
        mp = self.mp
        out = {}
        for n in self.M:
            v = J['lanes'].get(n)
            if not v or not v['pieces']:
                continue
            pts, bits = [mp.pt(*v['pieces'][0][:2])], []
            for x0, y0, x1, y1, L in v['pieces']:
                if math.hypot(x0 - pts[-1][0], y0 - pts[-1][1]) > 1e-6:
                    pts.append(mp.pt(x0, y0))
                    bits.append(mp.bit(0 if L == 'F.Cu' else 1))
                pts.append(mp.pt(x1, y1))
                bits.append(mp.bit(0 if L == 'F.Cu' else 1))
            out[n] = (pts, bits, None)
        return out

    def json_vias(self, J):
        return [(n, *self.mp.pt(x, y)) for n, x, y in J.get('vias', [])]

    def col_vias(self, lanes, plan):
        out = []
        for n, (P, _b, U) in lanes.items():
            for c in plan.changes.get(n, []):
                out.append((n, *along(P, U, c)))
        return out

    def _states(self):
        S = self.S
        raw = {}
        for i, p in enumerate(self.plans):
            raw[f'plan{i}'] = self.slots(p)
        raw['root'] = self.slots(self.root)
        raw['slots'] = self.slots(self.final)
        raw['p1'] = self.pass_lanes(0, self.final)
        raw['p2'] = self.pass_lanes(len(S.Gt['passes']) - 1, self.final)
        vias = {k: self.col_vias(v, self.plans[int(k[4:])] if k.startswith('plan') else
                                 (self.root if k == 'root' else self.final)) for k, v in raw.items()}
        for key, J in (('geo', S.geo), ('pol', S.pol), ('pairs', S.pairsd), ('held', S.held), ('snap', S.snapd)):
            if J:
                raw[key] = self.json_lanes(J)
                vias[key] = self.json_vias(J)
        for r in S.failed:
            J = json.load(open(os.path.join(S.loop, f'p{r["i"]}.json')))
            raw[f'fail{r["i"]}'] = self.json_lanes(J)
            vias[f'fail{r["i"]}'] = self.json_vias(J)
        # every state of a lane resampled to ONE count, from its longest: a morph is a point-for-point blend
        self.nres = {}
        for n in self.M:
            Lmax = max(plen(st[n][0]) for st in raw.values() if n in st)
            self.nres[n] = int(min(1600, max(120, Lmax / 0.03)))
        self.st = {}
        for key, st in raw.items():
            self.st[key] = {n: resample(P, bits, self.nres[n]) for n, (P, bits, _U) in st.items()}
        self.vias = vias
        self.raw = raw

    def _copper(self):
        S = self.S
        run = set(S.nets)
        self.base_c = copper(S.base, run)
        self.fo_c = copper(S.fo, run)
        self.seq_c = copper(S.seq, run)
        self.human_c = copper(S.human, run) if S.human is not None else None
        # the fanout's changes: stubs the ends model moved away (in the round's input, not in its fanout) and laid
        self.stub_gone = minus(self.base_c[0], self.fo_c[0], _skey)
        self.stub_new = minus(self.fo_c[0], self.base_c[0], _skey)
        vkey = lambda v: (round(v[0] * 1000), round(v[1] * 1000), v[3])
        self.via_gone = minus(self.base_c[1], self.fo_c[1], vkey)
        self.via_new = minus(self.fo_c[1], self.base_c[1], vkey)
        self.stub_kept = minus(self.fo_c[0], self.stub_new, _skey)
        self.via_kept = minus(self.fo_c[1], self.via_new, vkey)
        # the routed copper: what the route added to the fanout, per lane
        rs = minus(self.seq_c[0], self.fo_c[0], _skey)
        rv = minus(self.seq_c[1], self.fo_c[1], vkey)
        self.routed = {n: ([s for s in rs if S.lane_of.get(s[6]) == n], [v for v in rv if S.lane_of.get(v[3]) == n])
                       for n in self.M}
        others = set(n.name.split('/')[-1] for n in S.fo.nets.values()) - run
        self.other_c = copper(S.fo, others)
        # the source-side and destination-side halves of the new stubs
        DB = S.Gt['DB']
        near_dst = lambda x, y: DB[0] - 1.5 <= x <= DB[2] + 1.5 and DB[1] - 1.5 <= y <= DB[3] + 1.5
        self.teeth_new = [s for s in self.stub_new if not near_dst(s[0], s[1])]
        self.berths_new = [s for s in self.stub_new if near_dst(s[0], s[1])]
        self.tvia_new = [v for v in self.via_new if not near_dst(v[0], v[1])]
        self.bvia_new = [v for v in self.via_new if near_dst(v[0], v[1])]
        moved = sorted({s[6] for s in self.stub_gone} | {s[6] for s in self.teeth_new})
        self.moved_teeth = sorted({S.lane_of.get(m, m) for m in moved})
        # the numbers of the result, counted as whole_route.py counts them
        self.vias_ours = len(self.seq_c[1])
        self.mm_ours = copper_len(self.seq_c[0])
        if self.human_c:
            self.vias_human = len(self.human_c[1])
            self.mm_human = copper_len(self.human_c[0])


# =============================================================================== drawing
class Canvas:
    def __init__(self, film, base):
        from PIL import ImageDraw
        self.f = film
        self.img = base.copy()
        self.d = ImageDraw.Draw(self.img)
        self.clipped = False

    def clip(self):
        """the board's overlays end at its panel: everything outside it painted over, once a frame, before the
        panels and cards are drawn"""
        if self.clipped:
            return
        self.clipped = True
        for box in ((0, 0, W, BOARD[1]), (0, BOARD[3], W, H), (0, 0, BOARD[0], H), (BOARD[2], 0, W, H)):
            self.d.rectangle([v * SS for v in box], fill=BG)

    # ---------------------------------------------------------------- primitives (1x coordinates in, SS drawn)
    def text(self, x, y, s, px=20, col=TEXT, bold=False, mono=False, anchor='la'):
        self.d.text((x * SS, y * SS), s, font=font(px * SS, bold, mono), fill=col, anchor=anchor)

    def tlen(self, s, px=20, bold=False, mono=False):
        return font(px * SS, bold, mono).getlength(s) / SS

    def wrap(self, s, px, width, bold=False):
        out, cur = [], ''
        for w in s.split(' '):
            t = (cur + ' ' + w).strip()
            if self.tlen(t, px, bold) > width and cur:
                out.append(cur)
                cur = w
            else:
                cur = t
        return out + ([cur] if cur else [])

    def rect(self, box, fill=None, outline=None, width=1, r=0):
        b = [v * SS for v in box]
        if r:
            self.d.rounded_rectangle(b, radius=r * SS, fill=fill, outline=outline, width=int(width * SS))
        else:
            self.d.rectangle(b, fill=fill, outline=outline, width=int(width * SS))

    def card(self, box, alpha=0.86, border=(70, 78, 92)):
        from PIL import Image
        self.clip()
        b = tuple(int(v * SS) for v in box)
        reg = self.img.crop(b)
        self.img.paste(Image.blend(reg, Image.new('RGB', reg.size, PANEL), alpha), b[:2])
        self.rect(box, outline=border, width=1, r=8)

    def line(self, pts, col, w=1.0):
        self.d.line([(x * SS, y * SS) for x, y in pts], fill=col, width=max(1, int(round(w * SS))), joint='curve')

    # ---------------------------------------------------------------- board-space primitives
    def wline(self, pts, col, w_px, joint=True):
        f = self.f
        P = [f.X(x, y) for x, y in pts]
        if len(P) >= 2:
            self.d.line(P, fill=col, width=max(1, int(round(w_px * SS))), joint='curve' if joint else None)

    def wring(self, x, y, r_mm, col, w_px=1.5, fill=None):
        cx, cy = self.f.X(x, y)
        r = max(1.5 * SS, r_mm * self.f.sc * SS)
        self.d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=col, width=max(1, int(w_px * SS)), fill=fill)

    def wdot(self, x, y, r_px, col):
        cx, cy = self.f.X(x, y)
        r = r_px * SS
        self.d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)

    def wtext(self, x, y, s, px=12, col=TEXT, anchor='la', bold=False):
        cx, cy = self.f.X(x, y)
        self.d.text((cx, cy), s, font=font(px * SS, bold), fill=col, anchor=anchor)

    def lanes(self, st, alpha=1.0, w=2.6, only=None, dim=0.0, ribbon=False, hl=None):
        """lanes of a state {n: (points, bits)} -- B under F, pairs on a ribbon; `only` lanes at full strength,
        the rest at `dim`"""
        f = self.f
        prs = f.S.pairs
        order = [n for n in f.M if n in st]
        a_of = lambda n: alpha * (1.0 if only is None or n in only else dim)
        if ribbon:
            for n in order:
                if n in prs and a_of(n) > 0.02:
                    P = st[n][0]
                    wr = (f.rules['PP'] + f.rules['TW']) * f.sc
                    self.d.line([f.X(x, y) for x, y in P], fill=mix(BODY, (110, 92, 26), a_of(n)),
                                width=max(1, int(wr * SS)), joint='curve')
        for layer in (1, 0):
            for n in order:
                a = a_of(n)
                if a <= 0.02:
                    continue
                P, bits = st[n]
                col = mix(BODY, LAYC[layer], a)
                ww = w * (1.6 if hl and n in hl else 1.0)
                j0 = None
                for j in range(len(bits) + 1):
                    on = j < len(bits) and bits[j] == layer
                    if on and j0 is None:
                        j0 = j
                    elif not on and j0 is not None:
                        self.wline(P[j0:j + 1], col, ww)
                        j0 = None

    def vias(self, vs, alpha=1.0, only=None, dim=0.0, r_mm=None):
        f = self.f
        r = (r_mm if r_mm is not None else f.rules['via'] / 2)
        for n, x, y in vs:
            a = alpha * (1.0 if only is None or n in only else dim)
            if a > 0.02:
                self.wring(x, y, r, mix(BODY, VIA_COL, a), 1.6)

    def segs(self, segs, alpha=1.0, true_width=True, w_px=2.0, col=None):
        f = self.f
        for layer in ('B.Cu', 'F.Cu'):
            c = col or LAYC[0 if layer == 'F.Cu' else 1]
            c = mix(BODY, c, alpha)
            for s in segs:
                if s[5] != layer:
                    continue
                w = s[4] * f.sc if true_width else w_px
                self.d.line([f.X(s[0], s[1]), f.X(s[2], s[3])], fill=c, width=max(1, int(round(w * SS))))
                if true_width:
                    for (x, y) in ((s[0], s[1]), (s[2], s[3])):
                        cx, cy = f.X(x, y)
                        r = w * SS / 2
                        self.d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=c)

    def cvias(self, vias, alpha=1.0):
        for x, y, size, _n in vias:
            cx, cy = self.f.X(x, y)
            r = size / 2 * self.f.sc * SS
            self.d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=mix(BODY, (200, 200, 200), alpha))
            r2 = r * 0.55
            self.d.ellipse([cx - r2, cy - r2, cx + r2, cy + r2], fill=mix(BODY, (30, 30, 30), alpha))


def morph(A, B, t):
    """two resampled states, blended point for point (the layer of the nearer end)"""
    out = {}
    for n in A:
        if n not in B:
            continue
        (PA, bA), (PB, bB) = A[n], B[n]
        out[n] = (PA + (PB - PA) * t, bA if t < 0.5 else bB)
    return out


def morph_vias(VA, VB, t):
    """two via lists blended: each lane's vias matched in order along x; the unmatched fade (alpha returned)"""
    ga, gb = collections.defaultdict(list), collections.defaultdict(list)
    for n, x, y in VA:
        ga[n].append((x, y))
    for n, x, y in VB:
        gb[n].append((x, y))
    out = []
    for n in set(ga) | set(gb):
        a, b = sorted(ga[n]), sorted(gb[n])
        for j in range(max(len(a), len(b))):
            if j < len(a) and j < len(b):
                out.append((n, lerp(a[j][0], b[j][0], t), lerp(a[j][1], b[j][1], t), 1.0))
            elif j < len(a):
                out.append((n, a[j][0], a[j][1], 1.0 - t))
            else:
                out.append((n, b[j][0], b[j][1], t))
    return out


# =============================================================================== the braid panel
_STRANDS = {}


def braid_strands(film, plan, w=0.11):
    key = id(plan)
    if key in _STRANDS and _STRANDS[key][0] is plan:
        return _STRANDS[key][1]
    out = _braid_strands(film, plan, w)
    if plan in film.plans or plan is film.final or plan is film.root:
        _STRANDS[key] = (plan, out)
    return out


def _braid_strands(film, plan, w=0.11):
    """every lane's strand: u samples, y (rank) at each, layer bit at each -- a strand's rank is its launch rank
    plus a ramp of one rank at each of its crossings (down past a lane launched south of it, up past one launched
    north), so the strands are a consistent permutation everywhere"""
    T = film.S.T
    per = collections.defaultdict(list)
    li = film.li
    for k, u in plan.cross.items():
        a, b = tuple(k)
        per[a].append((u, 1 if li[a] < li[b] else -1))
        per[b].append((u, 1 if li[b] < li[a] else -1))
    out = {}
    for n in film.M:
        u0, u1 = T['entry'][n], T['end'][n]
        us = np.arange(u0, u1 + 1e-9, 0.025)
        if us[-1] < u1:
            us = np.append(us, u1)
        y = np.full(len(us), float(li[n]))
        for (ux, sg) in per[n]:
            y += sg * np.clip((us - ux) / (2 * w) + 0.5, 0.0, 1.0)
        ch = np.array(plan.changes.get(n, []), float)
        bits = film.tl[n] ^ (np.searchsorted(ch, us, side='left') & 1) if len(ch) else np.full(len(us), film.tl[n])
        out[n] = (us, y, bits)
    return out


def magnify_view(box, view, title):
    """the magnifier's world view: `view`'s centre and width, its height the box's own aspect"""
    bx0, by0, bx1, by1 = box[0] + 6, box[1] + (26 if title else 6), box[2] - 6, box[3] - 6
    cx, cy, hw = (view[0] + view[2]) / 2, (view[1] + view[3]) / 2, (view[2] - view[0]) / 2
    hh = hw * (by1 - by0) / (bx1 - bx0)
    return (cx - hw, cy - hh, cx + hw, cy + hh)


def magnify(C, film, box, view, draw_fn, title=None, frame=True):
    """a magnifier: a region of the board drawn into `box` on an image of its own (so it clips itself) -- its pads,
    then draw_fn(draw, Z, s): Z maps board mm to the magnifier's pixels, s its pixels per mm (both supersampled)"""
    from PIL import Image, ImageDraw
    C.card(box, alpha=0.95)
    view = magnify_view(box, view, title)
    bx0, by0, bx1, by1 = box[0] + 6, box[1] + (26 if title else 6), box[2] - 6, box[3] - 6
    wpx, hpx = int((bx1 - bx0) * SS), int((by1 - by0) * SS)
    s = wpx / (view[2] - view[0])
    img = Image.new('RGB', (wpx, hpx), BODY)
    d = ImageDraw.Draw(img)
    Z = lambda x, y: ((x - view[0]) * s, (y - view[1]) * s)
    for fp in film.S.fo.footprints.values():
        for p in fp.pads:
            if view[0] - 1 < p.global_x < view[2] + 1 and view[1] - 1 < p.global_y < view[3] + 1 and p.pad_type != 'np_thru_hole':
                (x0, y0), (x1, y1) = Z(p.global_x - p.size_x / 2, p.global_y - p.size_y / 2), Z(p.global_x + p.size_x / 2, p.global_y + p.size_y / 2)
                if (p.shape or '').lower() == 'circle':
                    d.ellipse([x0, y0, x1, y1], fill=(150, 131, 75))
                else:
                    d.rounded_rectangle([x0, y0, x1, y1], radius=min(x1 - x0, y1 - y0) * 0.25, fill=(150, 131, 75))
    draw_fn(d, Z, s)
    C.img.paste(img, (int(bx0 * SS), int(by0 * SS)))
    if title:
        C.text(box[0] + 12, box[1] + 6, title, 13, TEXT)
    if frame:
        (a0, b0), (a1, b1) = film.X(view[0], view[1]), film.X(view[2], view[3])
        if BOARD[0] * SS <= a0 and a1 <= BOARD[2] * SS:
            C.d.rectangle([a0, b0, a1, b1], outline=ACCENT, width=2 * SS)
    return view


def funnel(C, film, a=1.0):
    """the braid's u against the board's: the magnified trunk as a funnel from the board's panel to the braid's"""
    f = film
    y0, y1 = BOARD[3], BRAID[1]
    bx0, bx1 = f.board_x(f.ku[1]), f.board_x(f.ku[2])
    C.d.polygon([(bx0 * SS, y0 * SS), (bx1 * SS, y0 * SS), (f.kx[2] * SS, y1 * SS), (f.kx[1] * SS, y1 * SS)],
                fill=mix(BG, (40, 44, 34), a))
    for u, x in zip(f.ku, f.kx):
        C.line([(f.board_x(u), y0), (x, y1)], mix(BG, (110, 104, 70), a), 1)
    C.line([(bx0, y0 - 3), (bx0, y0), (bx1, y0), (bx1, y0 - 3)], mix(BG, ACCENT, a), 1.5)
    C.text((f.kx[1] + f.kx[2]) / 2, (y0 + y1) / 2, f'the trunk between the arrays, magnified x{f.mag:.1f}', 12,
           mix(BG, (200, 190, 130), a), anchor='mm')


def draw_braid(C, film, plan, alpha=1.0, cursor=None, cuts=None, labels=True, hl=None, marks=True, vias_alpha=1.0):
    f = film
    T = f.S.T
    C.card(BRAID, alpha=0.92)
    funnel(C, f)
    st = braid_strands(f, plan)
    X = f.bx
    # the face band: no crossing and no change there
    xb0, xb1 = X(T['S_FACE']), X(T['BAND'])
    C.rect((xb0, BRAID[1] + 4, xb1, BRAID[3] - 4), fill=(34, 38, 46))
    C.text((xb0 + xb1) / 2, BRAID[1] + 6, 'face band', 11, DIM, anchor='ma')
    xh = X(T['Hn'][next(iter(T['Hn']))]) if T['Hn'] else X(f.S.Gt['H0'])
    C.line([(xh, BRAID[1] + 16), (xh, BRAID[3] - 4)], (70, 76, 88), 1)
    C.text(xh + 4, BRAID[1] + 6, 'rings (handoff)', 11, DIM)
    # u ticks
    for u in range(int(math.ceil(f.u0)), int(f.u1) + 1):
        x = X(u)
        C.line([(x, BRAID[3] - 6), (x, BRAID[3] - 2)], (90, 96, 108), 1)
        if u % 5 == 0:
            C.text(x, BRAID[3] - 7, f'u = {u} mm', 10, DIM, anchor='mb')
    if cuts:
        for (n, ulo, uhi, kind) in cuts:
            if n not in st:
                continue
            y = f.rank_y(float(np.interp((ulo + uhi) / 2, st[n][0], st[n][1])))
            col = (255, 110, 60) if kind == 'island' else (255, 60, 200)
            C.rect((X(ulo), y - 4, X(uhi), y + 4), outline=col, width=1.5)
    for layer in (1, 0):
        for n in f.M:
            us, y, bits = st[n]
            a = alpha * (1.0 if not hl or n in hl else 0.25)
            col = mix(PANEL, LAYC[layer], a)
            pts = [(X(u), f.rank_y(r)) for u, r in zip(us, y)]
            j0 = None
            for j in range(len(bits) + 1):
                on = j < len(bits) and bits[j] == layer
                if on and j0 is None:
                    j0 = j
                elif not on and j0 is not None:
                    C.line(pts[max(0, j0 - 1):j + 1] if j0 else pts[j0:j + 1], col, 1.8 if n not in f.S.pairs else 3.2)
                    j0 = None
    # ends, labels, vias
    for n in f.M:
        us, y, _b = st[n]
        x0, y0 = X(us[0]), f.rank_y(y[0])
        C.d.ellipse([(x0 - 2.5) * SS, (y0 - 2.5) * SS, (x0 + 2.5) * SS, (y0 + 2.5) * SS], fill=mix(PANEL, TEXT, alpha))
        x1, y1 = X(us[-1]), f.rank_y(y[-1])
        C.d.ellipse([(x1 - 2.5) * SS, (y1 - 2.5) * SS, (x1 + 2.5) * SS, (y1 + 2.5) * SS], fill=mix(PANEL, TEXT, alpha))
        if labels:
            C.text(x0 - 5, y0, n, 10, mix(PANEL, PAIR_COL if n in f.S.pairs else DIM, alpha), anchor='rm')
    if marks:
        for n in f.M:
            us, y, _b = st[n]
            for c in plan.changes.get(n, []):
                r = float(np.interp(c, us, y))
                x, yy = X(c), f.rank_y(r)
                R = 4.2
                C.d.ellipse([(x - R) * SS, (yy - R) * SS, (x + R) * SS, (yy + R) * SS],
                            outline=mix(PANEL, VIA_COL, alpha * vias_alpha), width=int(1.6 * SS))
    if cursor is not None:
        x = X(cursor)
        C.line([(x, BRAID[1] + 14), (x, BRAID[3] - 8)], ACCENT, 1.2)
    return st


# =============================================================================== the scenes
class Scene:
    def __init__(self, key, stage, secs, fn, captions):
        self.key, self.stage, self.secs, self.fn, self.captions = key, stage, secs, fn, captions


def build_scenes(film, speed=1.0):
    S, T, Gt = film.S, film.S.T, film.S.Gt
    nl, npairs = len(film.M), len(S.pairs)
    nn = len(S.nets)
    plans = film.plan_meta
    W_OVER, W_V = T['W_OVER'], T['W_V']
    xo = T['xo']
    n_cross = len(T['windows'])
    cls = Gt['cls']
    nN = sum(1 for n in film.M if cls.get(n) == 'N')
    nS = sum(1 for n in film.M if cls.get(n) == 'S')
    nW = nl - nN - nS
    P1, P2 = Gt['passes'][0], Gt['passes'][-1]
    sc = []

    # ------------------------------------------------------------------ title
    def title(C, t, T_):
        C.d.rectangle([0, 0, W * SS, H * SS], fill=BG)
        a = ease(span(t, 0, 0.3))
        C.text(W / 2, 400, film.title, 64, mix(BG, TEXT, a), bold=True, anchor='mm')
        C.text(W / 2, 480, f'{nl} lanes ({nn} nets: {nl - npairs} single-ended, {npairs} differential pairs) from '
               f'{S.src} to {S.dst}', 28, mix(BG, DIM, a), anchor='mm')
        C.text(W / 2, 530, 'every lane\'s whole path planned before anything is routed -- then routed all at once',
               24, mix(BG, DIM, a), anchor='mm')
        C.text(W / 2, 600, f'{os.path.basename(S.base_path)}  ·  round {S.rn} of the fanout  ·  loop round {S.lr}  ·  '
               f'{os.path.basename(os.path.dirname(S.rd))}', 18, mix(BG, (110, 116, 128), a), anchor='mm')
        if not S.identical:
            C.text(W / 2, 660, 'NOTE: a re-run did not reproduce the chain\'s files (filmed with --force)', 20,
                   (255, 90, 90), anchor='mm')
    sc.append(Scene('title', None, 5, title, []))

    # ------------------------------------------------------------------ the bench
    def bench(C, t, T_):
        a = ease(span(t, 0.1, 0.5))
        for ref in (S.src, S.dst):
            for p in S.fo.footprints[ref].pads:
                if (p.net_name or '').split('/')[-1] in S.nets:
                    C.wring(p.global_x, p.global_y, 0.22, mix(BODY, ACCENT, a), 1.6)
        C.segs(film.other_c[0], alpha=0.22)
        for ref, lab in ((S.src, 'source'), (S.dst, 'destination')):
            fp = S.fo.footprints[ref]
            xs = [p.global_x for p in fp.pads]
            ys = [p.global_y for p in fp.pads]
            C.wtext((min(xs) + max(xs)) / 2, min(ys) - 0.35, f'{ref} ({lab})', 18, mix(BODY, TEXT, a), anchor='mb', bold=True)
    sc.append(Scene('bench', None, 5, bench, [
        (0, f'The bench. {nl} lanes to route from {S.src} to {S.dst}: the lit balls ({nn} nets, {npairs} of the lanes are '
            f'differential pairs). Every other net and part on the board is an obstacle.')]))

    # ------------------------------------------------------------------ the ends
    e0 = next((e for e in S.ends if e[0].startswith('on the teeth')), None)
    e1 = next((e for e in S.ends if e[0].startswith('teeth moved')), None)

    def ends(C, t, T_):
        a_old = 1.0 - ease(span(t, 0.28, 0.45))
        a_new = ease(span(t, 0.30, 0.48))
        a_b = ease(span(t, 0.52, 0.66))
        C.segs(film.other_c[0], alpha=0.22)
        C.segs(film.stub_kept, alpha=0.95)
        C.cvias(film.via_kept)
        if a_old > 0:
            C.segs(film.stub_gone, alpha=a_old)
            C.cvias(film.via_gone, a_old)
        if a_new > 0:
            C.segs(film.teeth_new, alpha=a_new)
            C.cvias(film.tvia_new, a_new)
            hl = 1.0 - ease(span(t, 0.5, 0.7))
            for s in film.teeth_new:
                C.wring(s[0], s[1], 0.18, mix(BODY, ACCENT, hl * a_new), 1.4)
        if a_b > 0:
            C.segs(film.berths_new, alpha=a_b)
            C.cvias(film.bvia_new, a_b)
        # the two orders, numbered
        a_o = ease(span(t, 0.7, 0.82))
        if a_o > 0:
            for n in film.M:
                tooth, _s, _l, berth = film.ends[n]
                C.wdot(*tooth, 3.2, mix(BODY, TEXT, a_o))
                C.wdot(*berth, 3.2, mix(BODY, TEXT, a_o))
        # the wiring diagram: each lane from its launch rank to its final rank, straight -- every crossing is a pair
        # of lanes whose two orders disagree
        a_w = ease(span(t, 0.74, 0.88))
        if a_w > 0:
            C.card(BRAID, alpha=0.92)
            funnel(C, film, a_w)
            xa, xb = film.bx(T['S_FACE']), film.bx(max(T['end'].values()))
            C.text(xa - 6, BRAID[1] + 8, 'teeth (launch order)', 12, DIM, anchor='ra')
            C.text(xb + 6, BRAID[1] + 8, 'berths (final order)', 12, DIM, anchor='la')
            pts = {}
            for n in film.M:
                ya, yb = film.rank_y(film.li[n]), film.rank_y(film.fi[n])
                pts[n] = ((xa, ya), (xb, yb))
                C.line([(xa, ya), (xb, yb)], mix(PANEL, PAIR_COL if n in S.pairs else (170, 178, 190), a_w), 1.4)
                C.text(xa - 6, ya, n, 10, mix(PANEL, DIM, a_w), anchor='rm')
                C.text(xb + 6, yb, n, 10, mix(PANEL, DIM, a_w), anchor='lm')
            k = 0
            for key in T['windows']:
                a_, b_ = key.split('|')
                (p0, p1), (q0, q1) = pts[a_], pts[b_]
                d1, d2 = (p1[0] - p0[0], p1[1] - p0[1]), (q1[0] - q0[0], q1[1] - q0[1])
                den = d1[0] * d2[1] - d1[1] * d2[0]
                if abs(den) < 1e-9:
                    continue
                s_ = ((q0[0] - p0[0]) * d2[1] - (q0[1] - p0[1]) * d2[0]) / den
                k += 1
                if k / n_cross <= span(t, 0.8, 0.96):
                    x, y = p0[0] + d1[0] * s_, p0[1] + d1[1] * s_
                    C.d.ellipse([(x - 2.6) * SS, (y - 2.6) * SS, (x + 2.6) * SS, (y + 2.6) * SS], fill=ACCENT)
            C.text((xa + xb) / 2, BRAID[3] - 8, f'{n_cross} pairs of lanes disagree between the two orders: '
                   f'{n_cross} crossings the route must make', 14, mix(PANEL, ACCENT, ease(span(t, 0.9, 1.0))), anchor='mb')
    cap_e = [(0, f'The ends. Each net leaves {S.src} through a short stub (its TOOTH) and arrives at {S.dst} through '
                 f'another (its BERTH). Here are the teeth as this round started.')]
    if e0 and e1:
        cap_e.append((0.26, f'The ends model searches the fanout\'s escape menus, judged by the whole route\'s own '
                            f'estimate: with the teeth as they were, {e0[1]} vias and {e0[2]} crossings predicted; '
                            f'moving {e1[0].split("(")[1].rstrip(")")} teeth, {e1[1]} vias and {e1[2]} crossings.'))
    cap_e.append((0.5, f'...and every berth at {S.dst} with them. The fanout lays exactly what the model chose, and '
                       f'it is judged as laid.'))
    cap_e.append((0.72, 'The ends decide the braid: the teeth\'s order round the source and the berths\' order round '
                        'the destination fix every crossing -- two lanes whose orders disagree must cross once.'))
    sc.append(Scene('ends', 'ends', 16, ends, cap_e))

    # ------------------------------------------------------------------ the frame
    def draw_fanout(C, alpha=0.9):
        C.segs(film.other_c[0], alpha=0.22)
        C.segs(film.fo_c[0], alpha=alpha)
        C.cvias(film.fo_c[1], alpha)

    def draw_frame(C, a, ticks=True, refs=0.0):
        mp = film.mp
        FRC = (210, 214, 226)
        nrm = np.array([-film.spd[1], film.spd[0]])
        # the face band: no crossing and no change there
        q = [film.sp0 + film.spd * u for u in (T['S_FACE'], T['BAND'])]
        band = [tuple(q[0] - nrm * 7), tuple(q[1] - nrm * 7), tuple(q[1] + nrm * 7), tuple(q[0] + nrm * 7)]
        C.d.polygon([film.X(*b_) for b_ in band], fill=mix(BODY, (60, 64, 76), a * 0.8))
        sp = [mp.pt(*p) for p in Gt['spine']]
        C.wline(sp, mix(BODY, FRC, a), 2.0)
        pm = film.sp0 + film.spd * ((T['BAND'] + Gt['H0']) / 2)
        C.wtext(*(pm + nrm * 0.3), 'trunk', 16, mix(BODY, FRC, a), anchor='mt', bold=True)
        for k, ring in Gt['rings'].items():
            R = [mp.pt(*p) for p in ring]
            C.wline(R, mix(BODY, FRC, a * 0.9), 2.0)
            far = max(R, key=lambda p_: p_[0])
            C.wtext(far[0] + 0.3, far[1], f'ring {k}', 16, mix(BODY, FRC, a), anchor='lm', bold=True)
        if ticks:
            for u in range(int(math.ceil(film.u0)), int(Gt['H0']) + 1):
                p = film.sp0 + film.spd * u
                nrm = np.array([-film.spd[1], film.spd[0]])
                q0, q1 = p - nrm * 0.12, p + nrm * 0.12
                C.wline([tuple(q0), tuple(q1)], mix(BODY, (190, 196, 210), a), 1.2)
                if u % 5 == 0:
                    C.wtext(*(p - nrm * 0.25), f'u={u}', 11, mix(BODY, (190, 196, 210), a), anchor='mb')
        if refs > 0:
            for n in film.M:
                pts = []
                for f_, k_, o_ in Gt['refs'][n]:
                    u_, b_, nv_ = film.cols[(f_, k_)]
                    pts.append((b_[0] + o_ * nv_[0], b_[1] + o_ * nv_[1]))
                C.wline(pts, mix(BODY, (200, 200, 120), refs * 0.6), 1.0)

    def frame(C, t, T_):
        draw_fanout(C)
        a = ease(span(t, 0.0, 0.3))
        draw_frame(C, a, refs=ease(span(t, 0.45, 0.65)) * (1 - ease(span(t, 0.9, 1.0))))
        C.card(BRAID, alpha=0.92)
        ab = ease(span(t, 0.2, 0.5))
        funnel(C, film, ab)
        X = film.bx
        for u in range(int(math.ceil(film.u0)), int(film.u1) + 1):
            x = X(u)
            C.line([(x, BRAID[1] + 16), (x, BRAID[3] - 10)], mix(PANEL, (40, 44, 52), ab), 1)
            if u % 5 == 0:
                C.text(x, BRAID[3] - 8, f'u = {u} mm', 10, mix(PANEL, DIM, ab), anchor='mb')
        xb0, xb1 = X(T['S_FACE']), X(T['BAND'])
        C.rect((xb0, BRAID[1] + 4, xb1, BRAID[3] - 18), fill=mix(PANEL, (34, 38, 46), ab))
        for n in film.M:
            x0, x1 = X(T['entry'][n]), X(T['end'][n])
            y0, y1 = film.rank_y(film.li[n]), film.rank_y(film.fi[n])
            C.line([(x0, y0), (x0 + 30, y0)], mix(PANEL, DIM, ab), 1.4)
            C.line([(x1 - 30, y1), (x1, y1)], mix(PANEL, DIM, ab), 1.4)
            C.text(x0 - 5, y0, n, 10, mix(PANEL, PAIR_COL if n in S.pairs else DIM, ab), anchor='rm')
        C.text(W / 2, BRAID[1] + 8, 'the solve\'s plane: u along the route (the trunk, then a ring) -- '
               'the lanes in ORDER, north to south', 13, mix(PANEL, TEXT, ab), anchor='ma')
    sc.append(Scene('frame', 'frame', 10, frame, [
        (0, f'The frame, read off the board alone: a straight TRUNK from the source\'s box through the destination\'s, '
            f'and a RING round the destination for each of its faces ({nN} lanes north, {nS} south, {nW} end on its '
            f'near face).'),
        (0.3, 'Every lane\'s route is ONE coordinate u: along the trunk, then round its ring. On the trunk u is '
              'distance along the board -- so the plane below shares the board\'s x axis.'),
        (0.45, 'Each lane also gets a taut REFERENCE path (yellow): which side of each obstacle it would pass. '
               'The geometry will start from it.')]))

    # ------------------------------------------------------------------ the solve
    np_ = len(plans)
    rdr = lambda p: (f'{p["over"]} net{"s" if p["over"] != 1 else ""} over two vias, ' if p['over'] else '') + \
        f'{p["vias"]} layer changes'
    log = [ln for r in T['runs'] for ln in r['log']]
    mdl = next((ln for ln in log if ln.startswith('#Model')), '')
    mv = re.search(r'var:(\d+)/(\d+)\s+constraints:(\d+)/(\d+)', mdl)
    # the search's events, in its log's order: each plan it found (#1, #2, ..) and each rise of its lower bound
    # (#Bound), with the best and the bound after it -- a plan with nothing left between best and bound is proved
    events = []
    for ln in log:
        m = re.match(r'#(Bound|\d+)\s+([\d.]+)s\s+best:(\S+)\s+next:\[([^\]]*)\]', ln)
        if not m:
            continue
        best = float(m.group(3))
        nx = m.group(4).split(',')
        lb = float(nx[0]) if nx[0] else best
        events.append({'plan': m.group(1) != 'Bound', 't': float(m.group(2)), 'best': best, 'lb': lb})
    t_end = max([p['t'] for p in plans] + [e['t'] for e in events] + [0.05])
    last = plans[-1]
    fl = lambda v: (v - last['over'] * W_OVER) / W_V          # objective in layer changes, the over-two count held

    def solve(C, t, T_):
        draw_fanout(C, 0.55)
        # which plan: plan i holds from its slot; the morphs between
        k_in = 0.14
        seg_ = (0.62 - k_in) / max(1, np_)
        pi = min(np_ - 1, max(0, int((t - k_in) / seg_))) if t >= k_in else 0
        loc = (t - k_in - pi * seg_) / seg_ if t >= k_in else 0.0
        mt = ease(span(loc, 0.0, 0.3)) if pi > 0 else 1.0
        A = film.plans[pi - 1] if pi > 0 else film.plans[0]
        B = film.plans[pi]
        cur = lerp_plan(A, B, mt)
        sA = film.st[f'plan{pi - 1}'] if pi > 0 else film.st['plan0']
        sB = film.st[f'plan{pi}']
        a_in = ease(span(t, 0.08, k_in + 0.02))
        st = morph(sA, sB, mt)
        C.lanes(st, alpha=a_in, w=2.2)
        C.vias([(n, x, y) for n, x, y, al in morph_vias(film.vias[f'plan{pi - 1}' if pi > 0 else 'plan0'],
                                                         film.vias[f'plan{pi}'], mt) if al > 0.5], alpha=a_in)
        # the rule, shown once proved: one crossing and one change, lit on the braid and on the board
        a_r = ease(span(t, 0.7, 0.76)) * (1 - ease(span(t, 0.97, 1.0)))
        draw_braid(C, film, cur, alpha=max(0.15, a_in))
        # the search card
        a_c = ease(span(t, 0.0, 0.08))
        box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 400, BOARD[1] + 300)
        C.card(box)
        x0, y0 = box[0] + 16, box[1] + 14
        C.text(x0, y0, 'CP-SAT', 20, ACCENT, bold=True)
        rows = [f'{n_cross} crossing positions t (u along the route)',
                f'{T["KMAX"]} layer-change slots c per lane',
                f'{T["triples"]} braid triples: every three lanes cross',
                '   in one consistent order',
                'crossing lanes on different layers',
                'each lane ends on its berth\'s layer',
                'crossings and changes keep their room',
                'no crossing or change in the face band']
        if mv:
            rows.append(f'{mv.group(2)} variables, {mv.group(4)} constraints')
        for j, r in enumerate(rows):
            C.text(x0, y0 + 32 + j * 19, r, 14, mix(PANEL, TEXT if j < 3 else DIM, a_c))
        C.text(x0, y0 + 32 + len(rows) * 19 + 6, 'minimise: nets over two vias, then vias', 14, mix(PANEL, ACCENT, a_c))
        # the search chart, event by event (CP-SAT's plans came within hundredths of a second of each other)
        a_g = ease(span(t, k_in - 0.06, k_in))
        box2 = (BOARD[2] - 440, BOARD[1] + 6, BOARD[2] - 6, BOARD[1] + 310)
        C.card(box2)
        gx0, gy0, gx1, gy1 = box2[0] + 58, box2[1] + 50, box2[2] - 24, box2[3] - 64
        C.text(box2[0] + 16, box2[1] + 12, f'the search: {t_end:.2f} s of real time', 16, mix(PANEL, TEXT, a_g), bold=True)
        C.text(box2[0] + 16, box2[1] + 32, 'layer changes (with the net over two held)', 11, mix(PANEL, DIM, a_g))
        vals = [fl(e['best']) for e in events if e['best'] < math.inf] + [fl(e['lb']) for e in events]
        vmax, vmin = max(vals) + 1, max(0, min(vals) - 2)
        gy = lambda v: gy1 - (v - vmin) / (vmax - vmin) * (gy1 - gy0)
        E = max(1, len(events))
        gxx = lambda j: gx0 + (j + 0.5) / E * (gx1 - gx0)
        C.line([(gx0, gy0), (gx0, gy1), (gx1, gy1)], mix(PANEL, DIM, a_g), 1)
        for v in range(int(math.ceil(vmin)), int(vmax) + 1, max(1, int((vmax - vmin) / 5))):
            C.text(gx0 - 6, gy(v), str(v), 11, mix(PANEL, DIM, a_g), anchor='rm')
        # each event's moment in the film: a plan at its slot, a bound raised just before the next plan
        when, jp = [], 0
        for e in events:
            if e['plan']:
                when.append(k_in + jp * seg_)
                jp += 1
            else:
                when.append(k_in - 0.05 if jp == 0 else k_in + (jp - 1) * seg_ + 0.7 * seg_)
        shown = [j for j, w in enumerate(when) if t >= w]
        pb, pl = [], []
        for j in shown:
            e = events[j]
            if e['best'] < math.inf:
                if pb:
                    pb.append((gxx(j), pb[-1][1]))
                pb.append((gxx(j), gy(fl(e['best']))))
            if pl:
                pl.append((gxx(j), pl[-1][1]))
            pl.append((gxx(j), gy(fl(e['lb']))))
        if pl:
            C.line(pl + [(gxx(shown[-1]) + 0.4 * (gx1 - gx0) / E, pl[-1][1])], mix(PANEL, (90, 230, 255), a_g), 2)
        if pb:
            C.line(pb + [(gxx(shown[-1]) + 0.4 * (gx1 - gx0) / E, pb[-1][1])], mix(PANEL, TEXT, a_g), 2)
        k_ = 0
        for j in shown:
            e = events[j]
            x = gxx(j)
            C.text(x, gy1 + 6, f'{e["t"]:.2f}s', 10, mix(PANEL, DIM, a_g), anchor='ma')
            if e['plan']:
                y = gy(fl(e['best']))
                C.d.ellipse([(x - 4) * SS, (y - 4) * SS, (x + 4) * SS, (y + 4) * SS], fill=TEXT)
                C.text(x + 7, y - 3, f'plan {k_ + 1}: {plans[k_]["vias"]}', 12, TEXT, anchor='lb')
                k_ += 1
        C.text(gx1, gy1 + 22, 'white: the best plan   cyan: the lower bound', 11, mix(PANEL, DIM, a_g), anchor='ra')
        if t >= 0.62:
            ap = ease(span(t, 0.62, 0.66))
            C.text((box2[0] + box2[2]) / 2, box2[3] - 20, f'best = bound: {last["vias"]} changes, PROVED optimal', 15,
                   mix(PANEL, ACCENT, ap), bold=True, anchor='mm')
        # the rules, lit on one crossing and one change
        if a_r > 0:
            fin = film.plans[-1]
            mid_u = (T['BAND'] + Gt['H0']) / 2
            trunk = [(k, u) for k, u in fin.cross.items() if not any(Gt['cls'].get(n) and u > T['Hn'].get(n, 1e9) for n in k)]
            if trunk:
                kx, ux = min(trunk, key=lambda e: abs(e[1] - mid_u))
                a_, b_ = sorted(kx)
                stb = braid_strands(film, fin)
                ys = [float(np.interp(ux, stb[n][0], stb[n][1])) for n in (a_, b_)]
                x, y = film.bx(ux), film.rank_y(sum(ys) / 2)
                R = 11
                C.d.ellipse([(x - R) * SS, (y - R) * SS, (x + R) * SS, (y + R) * SS], outline=mix(PANEL, ACCENT, a_r), width=2 * SS)
                la, lb = ('F' if fin.layer(a_, ux) == 0 else 'B'), ('F' if fin.layer(b_, ux) == 0 else 'B')
                C.text(x + 14, y - 14, f'{a_} on {la} crosses {b_} on {lb}', 13, mix(PANEL, ACCENT, a_r), anchor='lb')
                # ...and on the board: where the slots swap
                Pa, _ba, Ua = film.raw[f'plan{np_ - 1}'][a_]
                Pb, _bb, Ub = film.raw[f'plan{np_ - 1}'][b_]
                qa, qb = along(Pa, Ua, ux), along(Pb, Ub, ux)
                C.wring((qa[0] + qb[0]) / 2, (qa[1] + qb[1]) / 2, 0.45, mix(BODY, ACCENT, a_r), 2)
            chs = [(n, c) for n in film.M for c in fin.changes.get(n, [])]
            if chs:
                n, c = min(chs, key=lambda e: abs(e[1] - mid_u - 0.8))
                stb = braid_strands(film, fin)
                x, y = film.bx(c), film.rank_y(float(np.interp(c, stb[n][0], stb[n][1])))
                R = 11
                C.d.ellipse([(x - R) * SS, (y - R) * SS, (x + R) * SS, (y + R) * SS], outline=mix(PANEL, (90, 230, 255), a_r), width=2 * SS)
                C.text(x + 14, y + 14, f'{n} changes layer: a via', 13, mix(PANEL, (90, 230, 255), a_r), anchor='lt')
                P, _b, U = film.raw[f'plan{np_ - 1}'][n]
                q = along(P, U, c)
                C.wring(q[0], q[1], 0.45, mix(BODY, (90, 230, 255), a_r), 2)
    cap_s = [(0, f'The solve (CP-SAT) decides every crossing and every layer change, all lanes at once. Its variables '
                 f'are positions along u: where each of the {n_cross} crossings happens, and where each lane changes '
                 f'layer.'),
             (0.14, 'Below, each plan it finds as a braid: red F, blue B, a white ring is a via. Above, the same plan '
                    'on the board, each lane in its references\' slot in that order -- the solve knows ORDER and '
                    'LAYERS, not millimetres.'),
             (0.3, 'The search: ' + ' -> '.join(rdr(p) for p in plans) + '. The lower bound rises as it goes.'),
             (0.62, f'Best meets bound: {last["vias"]} layer changes, proved optimal. Only a plan proved optimal in '
                    f'its vias goes on to the geometry.'),
             (0.72, 'The rules can be seen: two strands that cross are on different layers (a via lets a lane cross '
                    'lanes on its own layer), and each lane ends on its berth\'s layer.')]
    sc.append(Scene('solve', 'solve', 26, solve, cap_s))

    # ------------------------------------------------------------------ the loop's failed rounds (when any)
    if S.failed:
        # each failed round: its smooth plan and what its audit found, what went back (side flips to the geometry;
        # island and via cuts and history to the solve), and the solve again -- the braid from the round's solve to
        # the next round's
        fails = []
        for r in S.failed:
            i = r['i']
            ld_ = lambda f: json.load(open(os.path.join(S.loop, f))) if os.path.isfile(os.path.join(S.loop, f)) else {}
            nxt = S.rounds.get(i + 1)
            gi, pi_ = ld_(f'g{i}.json'), ld_(f'p{i}.json')
            flips = sorted({tuple(x) for x in pi_.get('flips', [])} - {tuple(x) for x in gi.get('flips', [])})
            cuts, hot, slog = [], [], ''
            if nxt and re.match(r's\d+\.json$', os.path.basename(nxt['solve'])):
                j = re.match(r's(\d+)\.json', os.path.basename(nxt['solve'])).group(1)
                cj = ld_(f'cuts{j}.json')
                cuts = [(c['lane'], c['u_lo'], c['u_hi'], 'island') for c in cj.get('cuts', [])] + \
                       [(c['lane'], c['u'] - c['w'], c['u'] + c['w'], 'via') for c in cj.get('vcuts', [])]
                slog = _read(os.path.join(S.loop, f's{j}.log'))
            for f in (f'hp{i}.json', f'hq{i}.json', f'hs{i}.json'):
                hot += [film.mp.pt(h[0], h[1]) for h in ld_(f).get('hot', [])]
            finds = []
            for line in _read(os.path.join(S.loop, f'p{i}.audit')).splitlines():
                m = re.search(r'at \((-?\d+\.\d+),\s*(-?\d+\.\d+)\)', line) or re.search(r'^DIVE \S+\s+\(\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)\)', line)
                if m and re.match(r'^(PITCH|STATIC|DIVE|SHAPE) [A-Z]', line):
                    finds.append(film.mp.pt(float(m.group(1)), float(m.group(2))))
            A_ = json.load(open(r['solve']))
            B_ = json.load(open(nxt['solve'])) if nxt else A_
            pa, pb = Plan(A_['cross'], A_['changes'], film), Plan(B_['cross'], B_['changes'], film)
            moved = sum(1 for k in pa.cross if abs(pa.cross[k] - pb.cross.get(k, pa.cross[k])) > 1e-6)
            chm = sum(1 for n in film.M if pa.changes.get(n, []) != pb.changes.get(n, []))
            st_ = re.search(r'\[(OPTIMAL|FEASIBLE)\] (\d+)s vias (\d+)', slog)
            fails.append(dict(i=i, flips=flips, cuts=cuts, hot=hot, finds=finds, pa=pa, pb=pb, moved=moved, chm=chm,
                              solved=st_.groups() if st_ else None, nhist=len(hot)))

        def loop_back(C, t, T_):
            draw_fanout(C, 0.5)
            nf = len(fails)
            j = min(nf - 1, int(t * nf))
            F_ = fails[j]
            loc = t * nf - j
            key = f'fail{F_["i"]}'
            flipped = {n for n, _isl in F_['flips']}
            C.lanes(film.st[key], alpha=ease(span(loc, 0, 0.1)), w=2.0, hl=flipped)
            C.vias(film.vias[key])
            a_f = ease(span(loc, 0.08, 0.16))
            for x, y in F_['finds']:
                C.wring(x, y, 0.32, mix(BODY, (255, 0, 255), a_f), 2.5)
            for isl in Gt['islands']:
                if any(isl['label'] == il for _n, il in F_['flips']):
                    x0, y0 = film.mp.pt(isl['box'][0], isl['box'][1])
                    x1, y1 = film.mp.pt(isl['box'][2], isl['box'][3])
                    C.d.rectangle([film.X(min(x0, x1) - 0.1, min(y0, y1) - 0.1), film.X(max(x0, x1) + 0.1, max(y0, y1) + 0.1)],
                                  outline=mix(BODY, (150, 245, 120), a_f), width=2 * SS)
            a_h = ease(span(loc, 0.35, 0.45))
            for x, y in F_['hot']:
                C.wring(x, y, 0.62, mix(BODY, (255, 150, 40), a_h), 2)
            m = ease(span(loc, 0.6, 0.85))
            cur = lerp_plan(F_['pa'], F_['pb'], m) if m > 0 else F_['pa']
            draw_braid(C, film, cur, alpha=1.0, cuts=F_['cuts'] if a_h > 0 else None, hl=flipped if loc < 0.35 else None)
            box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 470, BOARD[1] + 250)
            C.card(box)
            x0, y0 = box[0] + 16, box[1] + 14
            C.text(x0, y0, f'loop round {F_["i"]}: not yet', 20, ACCENT, bold=True)
            lines = [('the smooth plan\'s audit found', TEXT, 0.08), (f'   {len(F_["finds"])} place(s) short (magenta)', (255, 120, 255), 0.08),
                     ('the polish found no room on a side:', TEXT, 0.16),
                     (f'   {len(F_["flips"])} side flip(s) -> the geometry', (150, 245, 120), 0.16),
                     ('back to the solve:', TEXT, 0.35),
                     (f'   {sum(1 for c in F_["cuts"] if c[3] == "island")} island cut(s), '
                      f'{sum(1 for c in F_["cuts"] if c[3] == "via")} via cut(s) (boxes, below)', (255, 150, 60), 0.35),
                     (f'   {F_["nhist"]} history place(s) priced (orange)', (255, 150, 40), 0.4)]
            if F_['solved']:
                lines += [(f'the solve again ({F_["solved"][1]} s): {F_["moved"]} crossings', ACCENT, 0.6),
                          (f'   and {F_["chm"]} lanes\' changes moved, {F_["solved"][2]} vias', ACCENT, 0.6)]
            for k_, (l_, col, a0) in enumerate(lines):
                C.text(x0, y0 + 34 + k_ * 21, l_, 14, mix(PANEL, col, ease(span(loc, a0, a0 + 0.05))))
        f0 = fails[0]
        sc.append(Scene('loop', 'solve', 12 * len(fails), loop_back, [
            (0, f'Loop round {f0["i"]} did not pass. Its smooth plan\'s audit found {len(f0["finds"])} place(s) short '
                f'(magenta), and the polish found {len(f0["flips"])} lane(s) with no room on their side of an island (green).'),
            (0.35 / len(fails), 'What goes back: the side flips to the geometry; to the solve, the islands and vias the geometry '
                                'could not give room (cuts: no crossing or change of that lane in those spans) and the places '
                                'found short, priced (history).'),
            (0.6 / len(fails), 'The solve again, warm, floored by the root\'s proof: the same number of vias, the crossings and '
                               'changes moved out of the places that were short -- and the geometry again, with the flips.')]))

    # ------------------------------------------------------------------ the geometry
    def forces(C, marks, a, wmax=5.0):
        for r, how, pts in sorted(marks, key=lambda m: m[0]['price']):
            col = mix(BODY, FORCE.get(r['kind'], (255, 255, 255)) if r['paid'] <= 1e-4 else (255, 40, 40), a)
            w = min(wmax, 1.2 + 1.3 * math.log10(1.0 + r['price']))
            if how == 'strut':
                C.wline(pts, col, w)
            elif how == 'wall':
                # the lane stands at a wall (a pad box, the board edge, an island's side): a stub toward it, the wall
                nv = np.array(film.mp.vec(*r['nv'])) * r['wall']
                p0 = np.array(pts[0])
                q = p0 + nv * 0.22
                t_ = np.array([-nv[1], nv[0]]) * 0.12
                C.wline([tuple(p0), tuple(q)], col, w)
                C.wline([tuple(q - t_), tuple(q + t_)], col, w)
            else:
                C.wring(*pts[0], 0.07 + 0.015 * w, col, 1.4)

    def cross_section(C, u, st_key, pass_i, a):
        """the LP column at u on the trunk: its variables (the present lanes' offsets), its priced rows"""
        k = int(round(u / Gt['G']))
        L = Gt['passes'][pass_i]['lanes']
        pres = [(n, v) for n in film.M for (f, kk, v) in L[n] if f == 'T' and kk == k]
        if not pres:
            return
        u_, b, nv = film.cols[('T', k)]
        box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 330, BOARD[3] - 6)
        C.card(box)
        C.text(box[0] + 14, box[1] + 12, f'one LP column: u = {u_:.2f} mm', 16, mix(PANEL, TEXT, a), bold=True)
        C.text(box[0] + 14, box[1] + 34, f'{len(pres)} variables: these lanes\' offsets', 13, mix(PANEL, DIM, a))
        os_ = [v for _n, v in pres]
        lo, hi = min(os_) - 0.3, max(os_) + 0.3
        y0, y1 = box[1] + 84, box[3] - 40
        sgn = -1 if nv[1] > 0 else 1                # north up
        oy = lambda v: y0 + (hi - v) / (hi - lo) * (y1 - y0) if sgn > 0 else y0 + (v - lo) / (hi - lo) * (y1 - y0)
        # the two layers side by side: the order binds a layer's OWN lanes only, so an F lane and a B lane may stand
        # at one offset (F over B, as a human stacks them); same-layer neighbours stand a bar apart
        xL = {0: box[0] + 130, 1: box[0] + 196}
        C.text(xL[0], box[1] + 58, 'F.Cu', 13, mix(PANEL, F_COL, a), bold=True, anchor='mm')
        C.text(xL[1], box[1] + 58, 'B.Cu', 13, mix(PANEL, B_COL, a), bold=True, anchor='mm')
        for x_ in xL.values():
            C.line([(x_, y0 - 6), (x_, y1 + 6)], mix(PANEL, (70, 76, 88), a), 1)
        lay = {n: film.final.layer(n, u_) for n, _v in pres}
        at = {n: v for n, v in pres}
        rows = [r for r in Gt['passes'][pass_i]['rows'] if r['f'] == 'T' and r['k'] == k]
        for r in rows:
            col = mix(PANEL, FORCE.get(r['kind'], TEXT), a)
            if r['kind'] in ('pitch', 'via', 'viavia') and len(r['lanes']) == 2 and all(n in at for n in r['lanes']):
                na, nb = r['lanes']
                w = min(5, 1.2 + 1.3 * math.log10(1 + r['price']))
                if r['kind'] == 'pitch':
                    xx = xL[lay[na]] + (8 if lay[na] == 0 else -8)
                    C.line([(xx, oy(at[na])), (xx, oy(at[nb]))], col, w)
                else:
                    C.line([(xL[lay[na]], oy(at[na])), (xL[lay[nb]], oy(at[nb]))], col, w)
        prs_ = set(S.pairs)
        for L_ in (0, 1):
            ls = sorted([(oy(v), n) for n, v in pres if lay[n] == L_])
            # the labels a line apart at least, each led to its dot
            ys = [y for y, _n in ls]
            for j in range(1, len(ys)):
                ys[j] = max(ys[j], ys[j - 1] + 13)
            over = ys[-1] - (y1 + 10) if ys else 0
            if over > 0:
                ys = [y - over for y in ys]
                for j in range(len(ys) - 2, -1, -1):
                    ys[j] = min(ys[j], ys[j + 1] - 13)
            for (y, n), yl in zip(ls, ys):
                x_ = xL[L_]
                C.d.ellipse([(x_ - 5) * SS, (y - 5) * SS, (x_ + 5) * SS, (y + 5) * SS], fill=mix(PANEL, LAYC[L_], a))
                xt = x_ - 14 if L_ == 0 else x_ + 14
                C.line([(x_ + (-6 if L_ == 0 else 6), y), (xt, yl)], mix(PANEL, (90, 96, 108), a), 1)
                C.text(xt - (2 if L_ == 0 else -2), yl, n, 12, mix(PANEL, PAIR_COL if n in prs_ else TEXT, a),
                       anchor='rm' if L_ == 0 else 'lm')
        C.text(box[0] + 14, box[3] - 30, 'magenta: at the bar   cyan: a via\'s room', 11, mix(PANEL, DIM, a))
        # the column on the board
        ends_ = [(b[0] + (lo - 0.2) * nv[0], b[1] + (lo - 0.2) * nv[1]), (b[0] + (hi + 0.2) * nv[0], b[1] + (hi + 0.2) * nv[1])]
        C.wline(ends_, mix(BODY, ACCENT, a), 1.2)

    # up close: the window of the trunk where the most rules bind (both passes), in the force magnifier's aspect
    fbox = (BOARD[2] - 450, BOARD[1] + 6, BOARD[2] - 6, BOARD[3] - 6)
    fw = 2.6
    fh = fw * (fbox[3] - fbox[1] - 32) / (fbox[2] - fbox[0] - 12)
    fpts = [film.mp.pt(*r['pts'][0]) for P_ in (P1, P2) for r in P_['rows']
            if r['kind'] in ('pitch', 'via', 'viavia', 'static', 'bound')]
    fwin = None
    if fpts:
        c_ = max(fpts, key=lambda p: sum(1 for q in fpts if abs(q[0] - p[0]) < fw / 2 and abs(q[1] - p[1]) < fh / 2))
        near = [q for q in fpts if abs(q[0] - c_[0]) < fw / 2 and abs(q[1] - c_[1]) < fh / 2]
        mx, my = sum(q[0] for q in near) / len(near), sum(q[1] for q in near) / len(near)
        fwin = (mx - fw / 2, my - fh / 2, mx + fw / 2, my + fh / 2)

    def force_zoom(C, st, vs, marks, a):
        """the window up close: every lane at its copper's width, the vias at theirs, and the rules that bind"""
        TW_, PP_ = film.rules['TW'], film.rules['PP']
        prs = set(S.pairs)
        top = sorted(marks, key=lambda m: -m[0]['price'])

        def draw(d, Z, s):
            v0, v1 = (fwin[0] - 1, fwin[1] - 1), (fwin[2] + 1, fwin[3] + 1)
            for layer in (1, 0):
                for n, (P, bits) in st.items():
                    if not ((P[:, 0] > v0[0]) & (P[:, 0] < v1[0]) & (P[:, 1] > v0[1]) & (P[:, 1] < v1[1])).any():
                        continue
                    wd = (PP_ + TW_ if n in prs else TW_) * s
                    col = mix(BODY, LAYC[layer], 0.55 if n in prs else 0.8)
                    j0 = None
                    for j in range(len(bits) + 1):
                        on = j < len(bits) and bits[j] == layer
                        if on and j0 is None:
                            j0 = j
                        elif not on and j0 is not None:
                            Q = simplify(P[j0:j + 1], 0.3 / s * SS)
                            d.line([Z(*q) for q in Q], fill=col, width=max(1, int(wd)), joint='curve')
                            j0 = None
            for n, x, y, al in vs:
                cx, cy = Z(x, y)
                r = film.rules['via'] / 2 * s
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(185, 185, 185), outline=(240, 240, 240), width=SS)
            lab = 0
            for r, how, pts in top:
                col = FORCE.get(r['kind'], TEXT) if r['paid'] <= 1e-4 else (255, 40, 40)
                w = SS * min(7.0, 1.5 + 1.8 * math.log10(1.0 + r['price']))
                if not any(fwin[0] - 0.3 < p[0] < fwin[2] + 0.3 and fwin[1] - 0.3 < p[1] < fwin[3] + 0.3 for p in pts):
                    continue
                if how == 'strut':
                    d.line([Z(*pts[0]), Z(*pts[1])], fill=col, width=int(w))
                    for q in pts:
                        cx, cy = Z(*q)
                        d.ellipse([cx - w, cy - w, cx + w, cy + w], fill=col)
                    at = ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
                elif how == 'wall':
                    nv = np.array(film.mp.vec(*r['nv'])) * r['wall']
                    p0 = np.array(pts[0])
                    q = p0 + nv * (TW_ / 2 + film.rules['CL'])
                    tt = np.array([-nv[1], nv[0]]) * 0.09
                    d.line([Z(*p0), Z(*q)], fill=col, width=int(w))
                    d.line([Z(*(q - tt)), Z(*(q + tt))], fill=col, width=int(w))
                    at = tuple(q)
                else:
                    cx, cy = Z(*pts[0])
                    R = 0.05 * s + w
                    d.ellipse([cx - R, cy - R, cx + R, cy + R], outline=col, width=int(w * 0.7))
                    at = pts[0]
                if lab < 6 and fwin[0] < at[0] < fwin[2] and fwin[1] < at[1] < fwin[3]:
                    cx, cy = Z(*at)
                    d.text((cx + 8, cy - 8), f'{r["price"]:.3g}', font=font(12 * SS, True), fill=col, anchor='lb')
                    lab += 1
        magnify(C, film, fbox, fwin, draw, 'up close: the rules that bind, and their prices')

    def geometry(C, t, T_):
        draw_fanout(C, 0.5)
        m1 = ease(span(t, 0.02, 0.16))
        m2 = ease(span(t, 0.72, 0.84))
        if t < 0.72:
            st = morph(film.st['slots'], film.st['p1'], m1)
            vs = morph_vias(film.vias['slots'], film.vias['p1'], m1)
            rows = P1['rows']
        else:
            st = morph(film.st['p1'], film.st['p2'], m2)
            vs = morph_vias(film.vias['p1'], film.vias['p2'], m2)
            rows = P2['rows']
        # the islands (another part's pads a lane must pass on one side) in pass 2
        a_i = ease(span(t, 0.68, 0.74))
        if a_i > 0:
            for isl in Gt['islands']:
                if isl['label'].startswith(('end ', 'tooth ', 'stub ', 'svia ')):
                    continue
                x0, y0 = film.mp.pt(isl['box'][0], isl['box'][1])
                x1, y1 = film.mp.pt(isl['box'][2], isl['box'][3])
                P = [film.X(min(x0, x1) - 0.08, min(y0, y1) - 0.08), film.X(max(x0, x1) + 0.08, max(y0, y1) + 0.08)]
                C.d.rectangle([P[0], P[1]], outline=mix(BODY, (255, 160, 40), a_i), width=int(1.5 * SS))
        C.lanes(st, w=2.2)
        C.vias([(n, x, y) for n, x, y, al in vs])
        a_f = ease(span(t, 0.2, 0.26)) * (1 - ease(span(t, 0.68, 0.72))) + ease(span(t, 0.86, 0.92))
        if a_f > 0:
            marks = force_marks(film, rows, st, vs)
            forces(C, marks, a_f)
            if a_f > 0.05 and fwin:
                force_zoom(C, st, vs, marks, a_f)
        # the cursor column
        cur = None
        if 0.42 <= t < 0.68:
            cur = T['BAND'] + (Gt['H0'] - T['BAND'] - 0.2) * ease(span(t, 0.43, 0.67))
            cross_section(C, cur, 'p1', 0, ease(span(t, 0.42, 0.45)) * (1 - ease(span(t, 0.66, 0.68))))
        draw_braid(C, film, film.final, alpha=0.9, cursor=cur)
        # the LP card
        if t < 0.42 or t >= 0.68:
            P_ = P1 if t < 0.72 else P2
            lp = [ln for ln in Gt['log'] if 'joint LP' in ln]
            ln = lp[0 if t < 0.72 else -1] if lp else ''
            m = re.search(r'build (\d+)s solve (\d+)s', ln)
            box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 400, BOARD[1] + 250)
            C.card(box)
            x0, y0 = box[0] + 16, box[1] + 14
            C.text(x0, y0, f'the LP, pass {1 if t < 0.72 else 2}', 20, ACCENT, bold=True)
            lines = [f'{P_["nvar"]} variables: an offset per lane', f'  per column (every {Gt["G"]:.2f} mm)',
                     f'{P_["nrow"]} rows (inequalities)', 'solved as its dual by HiGHS' + (f' in {m.group(2)} s' if m else ''),
                     f'{len(P_["rows"])} rules with a shadow price:']
            cnt = collections.Counter(r['kind'] for r in P_['rows'])
            name = {'pitch': 'neighbours at the bar', 'via': 'a via\'s room', 'viavia': 'via to via', 'bound': 'pad box / edge',
                    'static': 'an island\'s side', 'turn': 'turn limit', 'pturn': 'a pair\'s turn', 'pdive': 'a pair\'s dive run',
                    'slope': 'slope cap', 'approach': 'approach to its end'}
            for j, l_ in enumerate(lines):
                C.text(x0, y0 + 32 + j * 19, l_, 14, TEXT if j < 4 else DIM)
            yy = y0 + 32 + len(lines) * 19
            for kind, c_ in cnt.most_common(6):
                C.d.rectangle([(x0 + 6) * SS, (yy + 4) * SS, (x0 + 22) * SS, (yy + 12) * SS], fill=FORCE.get(kind, TEXT))
                C.text(x0 + 30, yy, f'{c_}  {name.get(kind, kind)}', 13, DIM)
                yy += 18
    paid2 = P2.get('paid', {})
    npaid = sum(paid2.values())
    cap_g = [(0, f'The geometry: ONE joint linear program turns the order into millimetres -- an offset for every '
                 f'lane at every column ({Gt["G"]:.2f} mm apart) along its frame, in the solve\'s order, on its '
                 f'solved layers.'),
             (0.16, f'Hard rules: same-layer neighbours a bar ({Gt["rules"]["P_MIN"]:.3f} mm) apart, a via\'s room '
                    f'round every change, off the pad boxes. Soft: the length a move adds, bends, and a comfortable '
                    f'pitch ({Gt["rules"]["P_COMF"]:.3f} mm) wherever there is room.'),
             (0.22, 'WHY each lane is where it is: the LP\'s dual gives every rule a shadow price. The lit rules are '
                    'the ones holding a lane in place -- thicker pushes harder. Every other rule has room to spare.'),
             (0.42, 'One column of the LP (the cursor): its variables are the offsets of the lanes there -- in the '
                    'braid\'s order at the same u below, now in millimetres, same-layer neighbours held a bar apart.'),
             (0.68, 'Pass 2 holds each lane to ONE side of every island (another part\'s pads, orange): one split per '
                    'island, in the lane order, the side with the room.'),
             (0.86, (f'What the LP had to pay (red) would go back to the solve as cuts: {npaid} rule(s) paid here'
                     + (', none of them a cut.' if not (S.geo.get('cuts') or S.geo.get('vcuts')) else '.'))
              if npaid else 'Nothing paid: every hard rule met, no cut for the solve.')]
    sc.append(Scene('geometry', 'geometry', 30, geometry, cap_g))

    # ------------------------------------------------------------------ the polish
    # the polish's largest move: each polished vertex's distance from the geometry's line (its dense samples)
    dmax = 0.0
    if 'geo' in film.st and S.pol:
        for n in film.M:
            if n in film.st['geo'] and n in film.raw.get('pol', {}):
                G_ = film.st['geo'][n][0]
                for q in film.raw['pol'][n][0]:
                    dmax = max(dmax, float(np.sqrt(((G_ - np.asarray(q)) ** 2).sum(axis=1).min())))

    def polish(C, t, T_):
        draw_fanout(C, 0.5)
        m = ease(span(t, 0.1, 0.6))
        st = morph(film.st['p2'], film.st['geo'], ease(span(t, 0.0, 0.1)))
        st = morph(st, film.st['pol'], m) if m > 0 else st
        C.lanes(st, w=2.2)
        C.vias([(n, x, y) for n, x, y, al in morph_vias(film.vias['geo'], film.vias['pol'], m)])
        draw_braid(C, film, film.final, alpha=0.6)
        if S.gate.get('smooth'):
            box = (BOARD[0] + 6, BOARD[1] + 6, BOARD[0] + 700, BOARD[1] + 60)
            C.card(box)
            C.text(box[0] + 14, box[1] + 12, 'audit of the smooth plan', 13, DIM)
            C.text(box[0] + 14, box[1] + 30, S.gate['smooth'][:88], 14, (150, 245, 120), mono=True)
    sc.append(Scene('polish', 'polish', 7, polish, [
        (0, f'The polish: the audit\'s own measures (pitch, via rooms, clearance to pads as KiCad draws them, turns) '
            f'met in board xy by small vertex moves, one LP per round -- here the largest move is {dmax:.3f} mm.'),
        (0.5, 'A lane with no room on its side of an island would be FLIPPED to the other and the geometry run '
              'again. The smooth plan passes the audit.')]))

    # ------------------------------------------------------------------ the snap
    pairs_ = set(S.pairs)
    singles = set(film.M) - pairs_
    grid = Gt['rules']['grid']

    def zoom(C, box, view, t_state, show, title):
        """the snap up close: the router's grid, and the lanes of the states `show` [(state, width, alpha)]"""
        def draw(d, Z, s):
            gx = math.ceil(view_[0][0] / grid) * grid
            while gx < view_[0][2]:
                gy = math.ceil(view_[0][1] / grid) * grid
                while gy < view_[0][3]:
                    x, y = Z(gx, gy)
                    d.rectangle([x - 1, y - 1, x + 1, y + 1], fill=(78, 86, 98))
                    gy += grid
                gx += grid
            TW_, PP_ = film.rules['TW'], film.rules['PP']
            for key, wid, al in show:
                st = film.raw[key]
                J = {'pairs': S.pairsd, 'held': S.held, 'snap': S.snapd}.get(key)
                if J and al > 0.5:
                    for n in S.pairs:
                        if n not in t_state or n not in st:
                            continue
                        P, _bits, _U = st[n]
                        d.line([Z(*q) for q in simplify(P, 0.3 / s * SS)], fill=mix(PANEL, (120, 100, 28), al),
                               width=max(1, int((PP_ + TW_) * s)), joint='curve')
                        v = J['lanes'].get(n, {})
                        legs = []
                        for e in v.get('ends', []) or []:
                            legs += [(leg, e['layer']) for leg in e.get('legs', [])]
                        c_ = v.get('cross')
                        if c_:
                            for side in ('P', 'N'):
                                legs += [(pts_, L_) for pts_, L_ in c_['legs'][side]]
                        for pts_, L_ in legs:
                            col = LAYC[film.mp.bit(0 if L_ == 'F.Cu' else 1)]
                            d.line([Z(*film.mp.pt(*q)) for q in pts_], fill=mix(PANEL, col, al), width=max(1, int(TW_ * s)),
                                   joint='curve')
                        if c_:
                            for x_, y_, _side in c_['vias']:
                                cx, cy = Z(*film.mp.pt(x_, y_))
                                r = film.rules['via'] / 2 * s
                                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(185, 185, 185), outline=(240, 240, 240), width=SS)
                for layer in (1, 0):
                    for n, (P, bits, _U) in st.items():
                        if n not in t_state:
                            continue
                        for j, bt in enumerate(bits):
                            if bt == layer:
                                d.line([Z(*P[j]), Z(*P[j + 1])], fill=mix(PANEL, LAYC[layer], al), width=max(1, int(wid * SS)))
                for n, x, y in film.vias[key]:
                    if n in t_state:
                        cx, cy = Z(x, y)
                        r = film.rules['via'] / 2 * s
                        d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=mix(PANEL, VIA_COL, al), width=2 * SS)
        view_ = [magnify_view(box, view, True)]
        magnify(C, film, box, view, draw, title)

    # where to look closely: the crossover (an opposite-hands pair), else the first pair's dive, else a via
    focus = None
    for n in S.pairs:
        c = S.snapd['lanes'].get(n, {}).get('cross') if S.snapd else None
        if c:
            # the crossover's whole shape: its legs and its barrels
            q = [pt for side in ('P', 'N') for pts_, _L in c['legs'][side] for pt in pts_] + [v[:2] for v in c['vias']]
            xs, ys = [pt[0] for pt in q], [pt[1] for pt in q]
            focus = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, n)
            break
    if focus is None and S.snapd and S.snapd.get('vias'):
        n, x, y = S.snapd['vias'][0]
        focus = (x, y, n)
    if focus:
        focus = film.mp.pt(focus[0], focus[1]) + (focus[2],)

    def snap(C, t, T_):
        draw_fanout(C, 0.5)
        mp_ = ease(span(t, 0.05, 0.28))
        mq = ease(span(t, 0.4, 0.55))
        ms = ease(span(t, 0.62, 0.85))
        pr = morph(film.st['pol'], film.st['pairs'], mp_)
        si = morph(film.st['pol'], film.st['held'], mq)
        si = morph(si, film.st['snap'], ms) if ms > 0 else si
        pr = morph(pr, film.st['snap'], ms) if ms > 0 else pr
        st = {n: (pr[n] if n in pairs_ else si[n]) for n in film.M if n in pr and n in si}
        dim = 0.25 if t < 0.36 else 1.0
        C.lanes(st, w=2.2, only=pairs_ if t < 0.36 else None, dim=dim, hl=pairs_ if t < 0.36 else None, ribbon=t >= 0.12)
        vs = film.vias['snap'] if t >= 0.62 else film.vias['pairs'] if t >= 0.2 else film.vias['pol']
        C.vias(vs, only=pairs_ if t < 0.36 else None, dim=dim)
        if focus:
            v = (focus[0] - 1.25, focus[1] - 0.5, focus[0] + 1.25, focus[1] + 0.5)
            box = (BRAID[0], BRAID[1], BRAID[0] + 620, BRAID[3])
            if t < 0.36:
                zoom(C, box, v, pairs_, [('pol', 1.2, 0.35), ('pairs', 2.4, 1.0)],
                     f'{focus[2]}: laid as the pair router moves (the dots: its {grid} mm grid)')
            else:
                zoom(C, box, v, set(film.M), [('held', 1.2, 0.35), ('snap', 2.4, 1.0)] if t >= 0.62 else
                     [('pol', 1.2, 0.35), ('held', 2.2, 1.0)], 'the lanes: smooth (faint) and on the router\'s grid')
            # a second magnifier: a single's via
            sv = [(n, x, y) for n, x, y in film.vias['snap'] if n in singles]
            if sv:
                n, x, y = sv[len(sv) // 2]
                v2 = (x - 1.25, y - 0.5, x + 1.25, y + 0.5)
                box2 = (BRAID[0] + 640, BRAID[1], BRAID[0] + 1260, BRAID[3])
                zoom(C, box2, v2, set(film.M), [('pol', 1.2, 0.35), ('snap' if t >= 0.62 else 'held', 2.4, 1.0)],
                     f'{n}: its via, and its neighbours')
            box3 = (BRAID[0] + 1280, BRAID[1], BRAID[2], BRAID[3])
            C.card(box3, alpha=0.95)
            lines = [('pairs laid first', S.gate.get('pairs held', '')), ('snapped', S.gate.get('snapped', ''))]
            yy = box3[1] + 12
            for lab, g in lines:
                C.text(box3[0] + 12, yy, lab, 13, DIM)
                for w_ in C.wrap(g, 12, box3[2] - box3[0] - 24)[:3]:
                    yy += 18
                    C.text(box3[0] + 12, yy, w_, 12, (150, 245, 120))
                yy += 28
            if S.lint:
                C.text(box3[0] + 12, yy, S.lint[-1], 14, (150, 245, 120), bold=True)
    xo_txt = f' {", ".join(xo)} swap{"s" if len(xo) == 1 else ""} its legs at its dive (a crossover), as a designer does.' if xo else ''
    sc.append(Scene('snap', 'snap', 20, snap, [
        (0, 'The snap makes the smooth plan octilinear on the router\'s grid. The PAIRS go first and alone, moving as '
            'the pair router does: 45-degree turns, a turning radius of straight after each, a dive only on a '
            'straight run.' + xo_txt),
        (0.36, 'The pairs are then HELD while the polish fits the singles round them...'),
        (0.6, f'...and every single is laid on the router\'s {grid} mm grid: a grid search in a band round its smooth '
              f'line, starting and ending where the router will.')]))

    # ------------------------------------------------------------------ the audit
    def audit(C, t, T_):
        draw_fanout(C, 0.5)
        C.lanes(film.st['snap'], w=2.2, ribbon=True)
        C.vias(film.vias['snap'])
        box = (W / 2 - 520, BOARD[1] + 150, W / 2 + 520, BOARD[1] + 450)
        a = ease(span(t, 0.05, 0.2))
        C.card(box, alpha=0.9 * a)
        C.text(W / 2, box[1] + 26, 'the audit, the gate and the lint', 24, mix(PANEL, ACCENT, a), bold=True, anchor='ma')
        rows = [('smooth plan', S.gate.get('smooth', '')), ('pairs held', S.gate.get('pairs held', '')),
                ('snapped plan', S.gate.get('snapped', '')), ('lint', S.lint[-1] if S.lint else '')]
        for j, (lab, g) in enumerate(rows):
            aj = ease(span(t, 0.15 + 0.12 * j, 0.25 + 0.12 * j))
            y = box[1] + 80 + j * 48
            C.text(box[0] + 30, y, lab, 18, mix(PANEL, DIM, aj))
            C.text(box[0] + 200, y, g[:84], 15, mix(PANEL, (150, 245, 120), aj), mono=True)
        C.text(W / 2, box[3] - 40, f'loop round {S.lr}: the plan passes -- every check, nothing waived', 18,
               mix(PANEL, TEXT, ease(span(t, 0.7, 0.8))), anchor='ma')
    sc.append(Scene('audit', 'audit', 7, audit, [
        (0, 'The audit installs the plan exactly as the router will get it and runs every check; the gate passes it '
            'only when complete and clean with nothing waived; the lint checks what the snap promised.'),
        (0.55, 'A failing round sends its findings back -- side flips to the geometry, cuts and history (the places '
               'it was short, priced) to the solve -- and the loop runs again; trouble at the ends goes back to the '
               'fanout.')]))

    # ------------------------------------------------------------------ the route
    order = [n for n in S.route_order if n in film.routed] + [n for n in film.M if n not in S.route_order]

    def route(C, t, T_):
        C.segs(film.other_c[0], alpha=0.22)
        C.segs(film.fo_c[0], alpha=0.9)
        C.cvias(film.fo_c[1])
        C.lanes(film.st['snap'], alpha=0.28, w=1.6, ribbon=False)
        k_ = t / 0.8 * len(order)
        for j, n in enumerate(order):
            a = min(1.0, max(0.0, k_ - j))
            if a <= 0:
                break
            segs, vias = film.routed[n]
            C.segs(segs, alpha=a)
            C.cvias(vias, a)
        C.card(BRAID, alpha=0.95)
        shown = [ln for j, ln in enumerate(S.route_lines) if j < k_]
        C.text(BRAID[0] + 12, BRAID[1] + 8, 'route_lanes.py --plan  (the production router, each lane in its band, '
               'all at once)', 13, DIM)
        cols_ = 3
        per = int(math.ceil(len(S.route_lines) / cols_))
        for j, ln in enumerate(shown):
            c_, r_ = divmod(j, per)
            p = ln.split()
            txt = f'{p[0]:<7} {"pair" if p[1] == "pair" else "single":<6} in band  vias {p[p.index("vias") + 1]}  len {p[p.index("len") + 1]} mm' if 'vias' in p and 'len' in p else ln[:60]
            C.text(BRAID[0] + 12 + c_ * 620, BRAID[1] + 32 + r_ * 22, txt, 13, (150, 245, 120), mono=True)
        if t > 0.85:
            a = ease(span(t, 0.85, 0.92))
            box = (W / 2 - 430, BOARD[1] + 10, W / 2 + 430, BOARD[1] + 70)
            C.card(box, alpha=0.9 * a)
            C.text(W / 2, box[1] + 30, f'{S.route_summary.replace("SUMMARY seq: ", "")}   ·   check_connected: '
                   f'{"all connected" if S.connected else "OPEN NETS"}   ·   check_drc: {"clean" if S.drc_clean else "VIOLATIONS"}',
                   18, mix(PANEL, (150, 245, 120) if S.connected and S.drc_clean else (255, 90, 90), a), bold=True, anchor='mm')
    sc.append(Scene('route', 'route', 16, route, [
        (0, 'The route: the production router on the plan, each lane held to its band -- the pairs first (their end '
            'connectors and crossover laid as planned), then the singles in the berths\' order.'),
        (0.85, 'A lane the router reports routed is not proof its net connects: the board is checked -- connectivity '
               'and DRC, at the routed clearance.')]))

    # ------------------------------------------------------------------ the result
    m_ = re.search(r'(\d+/\d+) in band', S.route_summary)
    band_ = m_.group(1) if m_ else '?'
    def result(C, t, T_):
        a_h = ease(span(t, 0.35, 0.45)) * (1 - ease(span(t, 0.75, 0.85))) if film.human_c else 0.0
        C.segs(film.other_c[0], alpha=0.22)
        if a_h < 1:
            C.segs(film.seq_c[0], alpha=1 - a_h)
            C.cvias(film.seq_c[1], 1 - a_h)
        if a_h > 0:
            C.segs(film.human_c[0], alpha=a_h)
            C.cvias(film.human_c[1], a_h)
        C.card(BRAID, alpha=0.95)
        x0 = BRAID[0] + 40
        C.text(x0, BRAID[1] + 20, 'on these nets, over the whole board (every via and mm of their copper)', 15, DIM)
        C.text(x0, BRAID[1] + 60, f'the whole route:  {film.vias_ours} vias,  {film.mm_ours:.0f} mm', 30,
               mix(PANEL, TEXT, 1 - 0.6 * a_h), bold=True)
        if film.human_c:
            C.text(x0, BRAID[1] + 110, f'the human:        {film.vias_human} vias,  {film.mm_human:.0f} mm', 30,
                   mix(PANEL, TEXT, 0.4 + 0.6 * a_h), bold=True)
        C.text(x0 + 1000, BRAID[1] + 60, 'showing: ' + ('the human\'s board' if a_h > 0.5 else 'the whole route'), 18,
               ACCENT)
    sc.append(Scene('result', 'route', 10, result, [
        (0, f'{band_} lanes routed in their bands, all at once; check_connected: '
            f'{"every net connected" if S.connected else "NETS OPEN"}; check_drc: {"clean" if S.drc_clean else "VIOLATIONS"}.' + (
            f' The whole route: {film.vias_ours} vias and {film.mm_ours:.0f} mm; the human\'s board on the same nets: '
            f'{film.vias_human} vias and {film.mm_human:.0f} mm (the human\'s copper includes its length-matching meanders).'
            if film.human_c else ''))]))

    # ------------------------------------------------------------------ the loop, summed up
    def coda(C, t, T_):
        C.d.rectangle([0, 0, W * SS, H * SS], fill=BG)
        a = ease(span(t, 0, 0.2))
        boxes = [('ends', 'fanout'), ('solve', 'CP-SAT'), ('geometry', 'LP'), ('polish', 'LP rounds'),
                 ('snap', 'grid search'), ('audit', 'gate, lint'), ('route', 'router')]
        x0, y0, bw, gap = 150, 420, 200, 30
        for j, (k, sub) in enumerate(boxes):
            x = x0 + j * (bw + gap)
            C.rect((x, y0, x + bw, y0 + 90), fill=PANEL, outline=mix(BG, ACCENT, a), width=2, r=10)
            C.text(x + bw / 2, y0 + 30, k, 24, mix(BG, TEXT, a), bold=True, anchor='mm')
            C.text(x + bw / 2, y0 + 62, sub, 16, mix(BG, DIM, a), anchor='mm')
            if j:
                C.line([(x - gap + 4, y0 + 45), (x - 6, y0 + 45)], mix(BG, DIM, a), 2)
        # the loop's arrows back
        def back(j0, j1, lab, dy):
            xa, xb = x0 + j0 * (bw + gap) + bw / 2, x0 + j1 * (bw + gap) + bw / 2
            C.line([(xa, y0 + 90), (xa, y0 + 90 + dy), (xb, y0 + 90 + dy), (xb, y0 + 96)], mix(BG, (255, 110, 60), a), 2)
            C.text((xa + xb) / 2, y0 + 96 + dy, lab, 16, mix(BG, (255, 110, 60), a), anchor='ma')
        back(5, 1, 'cuts + history: the solve again', 50)
        back(3, 2, 'side flips', 20)
        back(5, 0, 'ends crowded: the fanout again', 120)
        C.text(W / 2, 250, 'the whole route', 48, mix(BG, TEXT, a), bold=True, anchor='mm')
        C.text(W / 2, 320, 'each stage fed a MEASUREMENT of the one before -- a plan is routed only once the router\'s '
               'own rules say it can be', 22, mix(BG, DIM, a), anchor='mm')
    sc.append(Scene('coda', None, 6, coda, []))
    for s in sc:
        s.secs *= speed
    return sc


# =============================================================================== the frames
_FILM = None


def chrome(C, scene, t, T_total, t_abs, scenes):
    """header, caption, the stage timeline"""
    C.text(HEAD[0], HEAD[1] + 4, 'the whole route', 26, TEXT, bold=True)
    S = C.f.S
    C.text(HEAD[0] + 230, HEAD[1] + 10, f'{os.path.basename(S.base_path)} → {S.src} to {S.dst}, {len(C.f.M)} lanes', 18, DIM)
    if scene.stage:
        C.text(HEAD[2], HEAD[1] + 4, scene.stage, 26, ACCENT, bold=True, anchor='ra')
    cap = None
    for (t0, s) in scene.captions:
        if t >= t0:
            cap = (t0, s)
    if cap:
        a = ease(span(t, cap[0], cap[0] + 0.6 / max(1.0, scene.secs)))
        for j, ln in enumerate(C.wrap(cap[1], 21, CAP[2] - CAP[0])[:2]):
            C.text(CAP[0], CAP[1] + 4 + j * 27, ln, 21, mix(BG, TEXT, a))
    # the timeline
    x0, x1 = FOOT[0], FOOT[2]
    wch = (x1 - x0) / len(STAGES)
    for j, st in enumerate(STAGES):
        on = scene.stage == st
        done = scene.stage in STAGES and STAGES.index(scene.stage) > j
        C.rect((x0 + j * wch + 3, FOOT[1] + 4, x0 + (j + 1) * wch - 3, FOOT[3] - 16),
               fill=(60, 52, 20) if on else PANEL, outline=ACCENT if on else (50, 56, 66), width=1, r=6)
        C.text(x0 + j * wch + wch / 2, FOOT[1] + 19, st, 16, ACCENT if on else (TEXT if done else DIM), bold=on, anchor='mm')
    C.rect((x0, FOOT[3] - 8, x0 + (x1 - x0) * t_abs / T_total, FOOT[3] - 4), fill=(90, 96, 108))


_BASE = None


def base_image(film):
    """the static substrate: the board (outline, pads) in its panel, drawn once"""
    from PIL import Image
    sys.path.insert(0, PYR)
    from route_render import BoardRenderer
    S = film.S
    img = Image.new('RGB', (W * SS, H * SS), BG)
    r = BoardRenderer(S.fo, size=1000, supersample=1, show_zones=False, margin_frac=0.0)
    bw, bh = (BOARD[2] - BOARD[0]) * SS, (BOARD[3] - BOARD[1]) * SS
    r.set_canvas(bw, bh)
    # the renderer's view, the same as the film's camera (its Transform fits and centres as ours does)
    v = film.view
    r.set_view(v)
    img.paste(r._base, (BOARD[0] * SS, BOARD[1] * SS))
    # (check the two transforms agree: the renderer's point for the view's corner and ours)
    px = r.tf.pt(v[0], v[1])
    ox = film.X(v[0], v[1])
    assert abs(px[0] + BOARD[0] * SS - ox[0]) < 2 and abs(px[1] + BOARD[1] * SS - ox[1]) < 2, (px, ox)
    return img


def render_frame(i):
    film, scenes, fps, T_total, base = _FILM
    t_abs = i / fps
    acc = 0.0
    for sc in scenes:
        if t_abs < acc + sc.secs or sc is scenes[-1]:
            break
        acc += sc.secs
    t = min(1.0, (t_abs - acc) / sc.secs)
    C = Canvas(film, base)
    sc.fn(C, t, t_abs)
    chrome(C, sc, t, T_total, t_abs, scenes)
    return C.img.reduce(SS)


def frame_bytes(i):
    return render_frame(i).tobytes()


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--_trace':
        if sys.argv[2] == 'solve':
            _trace_solve(sys.argv[3], sys.argv[4])
        else:
            _trace_geo(sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])
        return
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('rundir')
    ap.add_argument('out', nargs='?')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument('--stills', help='write PNG stills into this directory (at --at seconds, or one per scene)')
    ap.add_argument('--at', help='comma list of times (s) for --stills')
    ap.add_argument('--human', help='the human\'s board, drawn at the end (default fb_t2q_human.kicad_pcb beside this script)')
    ap.add_argument('--speed', type=float, default=1.0, help='scale every scene\'s length')
    ap.add_argument('--title', default='The whole route', help='the title card\'s title')
    ap.add_argument('--force', action='store_true', help='film a run whose re-runs do not reproduce the chain\'s files')
    ap.add_argument('--scenes', action='store_true', help='list the scenes and their start times, and stop')
    a = ap.parse_args()
    t0 = time.time()
    S = load_story(a)
    film = Film(S)
    film.title = a.title
    scenes = build_scenes(film, a.speed)
    T_total = sum(s.secs for s in scenes)
    acc = 0.0
    for s in scenes:
        print(f'  {acc:6.1f} s  {s.key:9s} {s.secs:5.1f} s')
        acc += s.secs
    if a.scenes:
        return
    global _FILM
    base = base_image(film)
    _FILM = (film, scenes, a.fps, T_total, base)
    print(f'whole_movie: {T_total:.0f} s at {a.fps} fps, prepared in {time.time() - t0:.0f} s')
    if a.stills:
        os.makedirs(a.stills, exist_ok=True)
        if a.at:
            ts = [float(x) for x in a.at.split(',')]
        else:
            ts, acc = [], 0.0
            for s in scenes:
                ts.append(acc + s.secs * 0.75)
                acc += s.secs
        for tt in ts:
            img = render_frame(int(round(tt * a.fps)))
            p = os.path.join(a.stills, f'still_{tt:06.1f}.png')
            img.save(p)
            print('wrote', p)
        if not a.out:
            return
    if not a.out:
        raise SystemExit('whole_movie: name OUT.mp4 (or --stills DIR)')
    nfr = int(round(T_total * a.fps))
    cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(a.fps),
           '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', a.out]
    ff = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    import multiprocessing as mpr
    t1 = time.time()
    with mpr.get_context('fork').Pool(a.workers) as pool:
        for j, buf in enumerate(pool.imap(frame_bytes, range(nfr), chunksize=4)):
            ff.stdin.write(buf)
            if j % (a.fps * 10) == 0:
                print(f'  frame {j}/{nfr} ({time.time() - t1:.0f} s)', flush=True)
    ff.stdin.close()
    if ff.wait() != 0:
        raise SystemExit('whole_movie: ffmpeg failed')
    print(f'wrote {a.out}: {nfr} frames, {T_total:.0f} s, in {time.time() - t0:.0f} s')


if __name__ == '__main__':
    main()
