#!/usr/bin/env python3
"""A PAIR's layer cut on two routing layers holds the pair on the cut's layer across the island, as a single's does
(awx/whole_solve.py: the loop's `lcuts`, CHAN_CUTS, and the layer cuts' parity on two layers).

  python3 tests/test_622_pair_layer_cut.py

On two layers a lane's layer is one bit that each of its changes flips -- a pair's too: its legs run on one layer, its
P leg's, and each of its changes is a dive. The loop gives a lane the audit finds against an island on one layer a
layer cut (whole_route.audit_layer_cuts), and the solve holds it on the other layer across the island. A pair's cut was
read on more layers than two and skipped on two: a strobe pair found against a passive's pad there was given its cut
every round, the solve planned the same, and the loop stopped NOT CONVERGING on the same finding, its plan walling a
ball in.

On a generated two-layer bus with one pair, its ends laid by one round of the whole route (ENDS_ONLY), this pins:

1. the bench's own plan: a plan, and the probe place on each lane -- a single's and the pair's -- a stretch of its trunk
   clear of its changes, on some layer L;
2. a single's FIRM layer cut there, on the other layer: the plan holds the single on that layer across the probe's box
   and breaks no firm cut (the cut as before: the control that the probe's box is placed on the lane);
3. the pair's FIRM layer cut there, on the other layer: the plan holds the PAIR on that layer across the box, breaks no
   firm cut, and the solve lists the pair's cut among its layer cuts.
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

try:
    from ortools.sat.python import cp_model  # noqa: F401
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
LAYERS = 'F.Cu,B.Cu'
HALF = 0.15                       # the probe box's half size, mm


def child_main():
    """(in a child: the bench from the environment, whole_solve imported under it) the probes' plans, as JSON"""
    sys.path.insert(0, AWX)
    sys.path.insert(1, os.path.join(ROOT, 'py_router'))
    os.chdir(AWX)
    import whole_ctx
    import whole_frame
    import whole_solve as ws
    dest = os.environ['DEST']
    with contextlib.redirect_stdout(io.StringIO()):
        ctx, _cs = whole_ctx.plan()
        Fr = whole_frame.build(ctx, dest)
        J0 = ws.solve(ctx, dest)
    out = {'plan0': J0 is not None}
    if J0 is None:
        print(json.dumps(out))
        return
    M = list(Fr.M)
    prs = getattr(ctx, 'pairs', {}) or {}
    pair = next((n for n in M if n in prs), None)
    single = next((n for n in M if n not in prs), None)
    out['pair'], out['single'] = pair, single
    bit = lambda n: 1 if ctx.tooth_layer[n] == 'B.Cu' else 0
    lay = lambda J, n, u: bit(n) ^ (sum(1 for c in J['changes'][n] if c < u) & 1)
    td = os.environ['_PAIRCUT_DIR']
    for tag, n in (('single', single), ('pair', pair)):
        if n is None:
            continue
        # the probe: the middle of the longest stretch of the lane's trunk between its entry, its changes and its end
        ev = sorted([J0['entry'][n], J0['tend'][n]] + [c for c in J0['changes'][n]
                                                       if J0['entry'][n] < c < J0['tend'][n]])
        lo_, hi_ = max(zip(ev, ev[1:]), key=lambda ab: ab[1] - ab[0])
        u = (lo_ + hi_) / 2
        L0 = lay(J0, n, u)
        x, y = Fr.spine.xy(u, whole_frame.ref(Fr, n, u))
        box = [round(x - HALF, 4), round(y - HALF, 4), round(x + HALF, 4), round(y + HALF, 4)]
        cf = os.path.join(td, f'cut_{tag}.json')
        json.dump({'cuts': [], 'vcuts': [], 'lcuts': [{'lane': n, 'island': 'PROBE', 'layer': 1 - L0, 'box': box,
                                                        'firm': True}]}, open(cf, 'w'))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            J1 = ws.solve(ctx, dest, cuts=[cf])
        said = [ln.strip() for ln in buf.getvalue().splitlines() if 'layer cuts (islands in the way)' in ln]
        out[tag] = {'lane': n, 'u': u, 'stretch': [lo_, hi_], 'L0': L0, 'plan': J1 is not None, 'said': said,
                    'layers': [lay(J1, n, u + d_) for d_ in (-HALF, 0.0, HALF)] if J1 is not None else None,
                    'firm_broken': (J1 or {}).get('firm_broken')}
    print(json.dumps(out))


def main():
    print('=' * 60)
    print('a pair\'s layer cut on two layers: held as a single\'s')
    print('=' * 60)
    fails = []
    keep = os.environ.get('PAIRCUT_TEST_DIR')           # (a directory to work in and keep, to look at afterwards)
    if keep:
        os.makedirs(keep, exist_ok=True)
    with (tempfile.TemporaryDirectory() if not keep else contextlib.nullcontext(keep)) as td:
        raw, bench0 = os.path.join(td, 'raw.kicad_pcb'), os.path.join(td, 'bench.kicad_pcb')
        e = {k: v for k, v in os.environ.items() if not k.startswith('_PAIRCUT')}
        e.update(ROUTE_LAYERS=LAYERS, STAGE_CACHE='0', TAUT_MEMO='0', OMP_NUM_THREADS='1')
        for cmd in (['synth_bus.py', raw, '--k', '6', '--pairs', '1', '--gap', '5.0', '--copper', '2'],
                    ['make_bench.py', raw, 'SU1', 'SD1', bench0]):
            r = subprocess.run([sys.executable] + cmd, cwd=AWX, capture_output=True, text=True, timeout=600, env=e)
            if r.returncode != 0:
                raise SystemExit(f'BROKEN TEST: {cmd[0]} failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
        run = os.path.join(td, 'run')
        r = subprocess.run([sys.executable, 'whole_route.py', '6', run, '1'], cwd=AWX, capture_output=True,
                           text=True, timeout=1800, env=dict(e, BASE=bench0, DEST='SD1', ENDS_ONLY='1'))
        bench = os.path.join(run, 'r1', 'fo.kicad_pcb')
        if not (os.path.isfile(bench) and os.path.isfile(os.path.join(run, 'r1', 'solve.json'))):
            raise SystemExit(f'BROKEN TEST: one round laid no ends or no plan:\n{r.stdout[-2000:]}{r.stderr[-2000:]}')
        nets = [ln.strip() for ln in open(os.path.join(run, 'r1', 'nets.lines')) if ln.strip()]
        ce = dict(e, BENCH=bench, NETS=','.join(nets), DEST='SD1', BRAID_EXACT_PAGES='0', _PAIRCUT_CHILD='1',
                  _PAIRCUT_DIR=td)
        r = subprocess.run([sys.executable, os.path.abspath(__file__)], env=ce, capture_output=True, text=True,
                           timeout=1800)
        if r.returncode != 0 or not r.stdout.strip():
            raise SystemExit(f'BROKEN TEST: the child died:\n{r.stdout[-1500:]}{r.stderr[-2000:]}')
        got = json.loads(r.stdout.strip().splitlines()[-1])
        # 1
        if not got.get('plan0'):
            raise SystemExit('BROKEN TEST: the generated bench has no plan as it is')
        if not got.get('pair') or not got.get('single'):
            raise SystemExit(f'BROKEN TEST: the bench\'s lanes hold no pair and a single: {got}')
        for tag in ('single', 'pair'):
            g = got[tag]
            if g['stretch'][1] - g['stretch'][0] < 4 * HALF:
                raise SystemExit(f'BROKEN TEST: the {tag}\'s trunk has no stretch clear of its changes for the probe: '
                                 f'{g}')
        # 2
        g = got['single']
        want = [1 - g['L0']] * 3
        if not g['plan'] or g['layers'] != want or g['firm_broken']:
            raise SystemExit(f'BROKEN TEST: a single\'s firm layer cut is not held across the probe (the probe is not '
                             f'on its lane): {g}')
        print(f"ok   2. the single {g['lane']} held on {'FB'[1 - g['L0']]} across the probe at u {g['u']:.2f}")
        # 3
        g = got['pair']
        want = [1 - g['L0']] * 3
        if not g['plan']:
            fails.append(f'3. the pair\'s cut left no plan: {g}')
        elif g['layers'] != want:
            fails.append(f"3. the pair {g['lane']} on {''.join('FB'[b_] for b_ in g['layers'])} across the probe at u "
                         f"{g['u']:.2f}, want it held on {'FB'[1 - g['L0']]} (its cut not read)")
        elif g['firm_broken']:
            fails.append(f"3. the pair's firm cut broken: {g['firm_broken']}")
        elif not any(f"{g['lane']} on {'FB'[1 - g['L0']]} under PROBE" in s_ for s_ in g['said']):
            fails.append(f"3. the solve does not list the pair's layer cut: {g['said']}")
        else:
            print(f"ok   3. the pair {g['lane']} held on {'FB'[1 - g['L0']]} across the probe at u {g['u']:.2f}")
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: on two layers a pair\'s layer cut holds the pair on the cut\'s layer across the island, as a '
          'single\'s holds the single')
    return 0


if __name__ == '__main__':
    if os.environ.get('_PAIRCUT_CHILD'):
        child_main()
        sys.exit(0)
    sys.exit(main())
