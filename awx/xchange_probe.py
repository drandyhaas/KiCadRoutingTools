"""xchange_probe.py -- how much two-via capacity a MID-CORRIDOR layer change
buys, on fixed ends (item 1 of the #622 s7 list, measured before it is built).

Runs the braid's plan phase on BOARD for the K nets (the sidecar beside the
board is read as the braid reads it; --one-corridor drops a sidecar's
'corridors' grouping so the human's ends are one corridor like ours), takes
each corridor's launch/target offsets and tooth/berth layers, and on the SAME
crossings (two ribbon lines inverted between launch and target cross at
f = dl / (dl - dt) of the corridor) solves four models with CP-SAT, each lane
a layer profile along its crossing sequence, each crossing's two lanes on
different layers, total vias (end vias T+D, plus corridor changes) <= CAP
or the lane is RESIDUE:
   pages    one layer through the region (today's plan AND today's braid)
   half     one layer for crossings before f=0.5, one after (item 1 as
            written: the change at the midpoint)
   quarter  one layer per quarter of the corridor
   free     any step function (one_change.py, session 3)
Minimises 1000 * residue + vias. usage:
   xchange_probe.py BOARD K [--cap 2] [--one-corridor] [--drop A,B] [--show]
"""
import sys, os, subprocess, time, json
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE); sys.path.insert(0, '.')
import braid as B
from ortools.sat.python import cp_model

board, K = sys.argv[1], sys.argv[2]
cap = int(sys.argv[sys.argv.index('--cap') + 1]) if '--cap' in sys.argv else 2
drop = set(sys.argv[sys.argv.index('--drop') + 1].split(',')) if '--drop' in sys.argv else set()
show = '--show' in sys.argv
names = [n for n in subprocess.run([sys.executable, 'coherent_nets.py', K], capture_output=True, text=True)
         .stdout.strip().split(',') if n and n not in drop]
log = lambda m='': None
plan = None
pj = os.path.splitext(board)[0] + '.plan.json'
if '--one-corridor' in sys.argv and os.path.exists(pj):
    plan = json.load(open(pj))
    plan.pop('corridors', None)
    if not plan.get('ends'):
        plan = {}          # not None: None makes setup reload the sidecar (with its corridors)
ctx, groups = B.setup(board, names, 'DU1', log, plan=plan)
corridors = [B.Corridor(ci, M, ctx, log) for ci, M in enumerate(groups)]
ctx.corridors = corridors
for c in corridors:
    try:
        c.run(plan_only=True)
    except Exception as e:
        print('corridor', c.idx, 'not planned:', e)

MODELS = ('pages', 'half', 'quarter', 'free')


def solve(M, lo, to, T, D, model, cap):
    """residue list, vias, changes, per-lane profile strings."""
    xs = {nm: [] for nm in M}
    for i, a in enumerate(M):
        for b in M[i + 1:]:
            dl = lo[b] - lo[a]; dt = to[b] - to[a]
            if dl * dt < 0:
                f = dl / (dl - dt)
                xs[a].append((f, b)); xs[b].append((f, a))
    for nm in M:
        xs[nm].sort()
    idx = {nm: {om: k for k, (f, om) in enumerate(xs[nm])} for nm in M}
    m = cp_model.CpModel()
    lay = {nm: [m.NewBoolVar(f'l_{nm}_{k}') for k in range(len(xs[nm]))] for nm in M}
    res = {nm: m.NewBoolVar(f'r_{nm}') for nm in M}
    # the model's SEGMENTS: crossings in one segment share a layer variable
    nseg = {'pages': 1, 'half': 2, 'quarter': 4, 'free': 0}[model]
    if nseg:
        for nm in M:
            segv = [m.NewBoolVar(f'p_{nm}_{q}') for q in range(nseg)]
            for k, (f, om) in enumerate(xs[nm]):
                q = min(nseg - 1, int(f * nseg))
                m.Add(lay[nm][k] == segv[q])
            lay[nm + '#seg'] = segv
    vias = {}
    for nm in M:
        if nseg:
            seq = [T[nm]] + lay[nm + '#seg'] + [D[nm]]
        else:
            seq = [T[nm]] + lay[nm] + [D[nm]]
        cs = []
        for k in range(len(seq) - 1):
            a, b = seq[k], seq[k + 1]
            if isinstance(a, int) and isinstance(b, int):
                cs.append(int(a != b)); continue
            d = m.NewBoolVar(f'c_{nm}_{k}')
            m.Add(d >= a - b); m.Add(d >= b - a); m.Add(d <= a + b); m.Add(d <= 2 - a - b)
            cs.append(d)
        vias[nm] = T[nm] + D[nm] + sum(cs)
        m.Add(vias[nm] <= cap + 8 * res[nm])
    for a in M:
        for (f, b) in xs[a]:
            if a < b:
                # a residue lane is a swimmer: no layer promise, its crossings unconstrained
                m.Add(lay[a][idx[a][b]] + lay[b][idx[b][a]] == 1).OnlyEnforceIf([res[a].Not(), res[b].Not()])
    m.Minimize(1000 * sum(res.values()) + sum(vias[nm] for nm in M))
    s = cp_model.CpSolver(); s.parameters.max_time_in_seconds = 60; s.parameters.num_workers = 8
    st = s.Solve(m)
    R = [nm for nm in M if s.Value(res[nm])]
    V = sum(s.Value(vias[nm]) if not isinstance(vias[nm], int) else vias[nm] for nm in M if nm not in R)
    nx = sum(len(v) for v in xs.values()) // 2
    near_mid = sum(1 for nm in M for (f, om) in xs[nm] if abs(f - 0.5) < 0.05) // 2
    prof = {}
    for nm in M:
        if nseg:
            prof[nm] = ''.join('B' if s.Value(v) else 'F' for v in lay[nm + '#seg'])
        else:
            prof[nm] = ''.join('B' if s.Value(v) else 'F' for v in lay[nm])
    return R, V, nx, near_mid, prof, s.StatusName(st)


tot = {mo: [0, 0] for mo in MODELS}
N = 0
for c in corridors:
    M = [nm for nm in c.members if nm in getattr(c, 'launch_o', {}) and nm in getattr(c, 'target_o', {})]
    if len(M) < 2:
        continue
    lo = {nm: c.launch_o[nm] for nm in M}; to = {nm: c.target_o[nm] for nm in M}
    T = {nm: int(ctx.tooth_layer[nm] == 'B.Cu') for nm in M}
    D = {nm: int(ctx.dest_layer[nm] == 'B.Cu') for nm in M}
    sc = getattr(c, 'sched_cur', None)
    braid_sw = [nm for nm in M if sc is not None and sc.page.get(nm) is None]
    N += len(M)
    print(f'{os.path.basename(board)} K={K} corridor {c.idx}: n={len(M)} tooth B {sum(T.values())} berth B {sum(D.values())}; '
          f'the braid\'s schedule swims {len(braid_sw)} {braid_sw}')
    for mo in MODELS:
        t0 = time.time()
        R, V, nx, nm_, prof, stn = solve(M, lo, to, T, D, mo, cap)
        tot[mo][0] += len(R); tot[mo][1] += V
        print(f'   {mo:8s} cap {cap}: RESIDUE {len(R):2d} {R}  vias {V:3d} ({nx} crossings, {nm_} within 5% of the midpoint) '
              f'{stn} {time.time() - t0:.1f}s')
        if show:
            for nm in M:
                print(f'      {nm:6s} T={"B" if T[nm] else "F"} D={"B" if D[nm] else "F"} {prof[nm]} {"RESIDUE" if nm in R else ""}')
print(f'TOTAL n={N}: ' + '; '.join(f'{mo} residue {tot[mo][0]} vias {tot[mo][1]}' for mo in MODELS))
