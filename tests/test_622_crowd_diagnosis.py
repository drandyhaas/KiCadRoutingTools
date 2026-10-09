#!/usr/bin/env python3
"""A whole solve with NO plan names the lanes whose crossings the ends leave no room for (whole_solve's CROWD
diagnosis).

  python3 tests/test_622_crowd_diagnosis.py

The zynq LVDS bus on four layers: rounds 1 and 3 of the whole route had no plan from the solve, and the round's
feedback named the lanes the ENDS MODEL names (its nets over two vias, its loaded or most crossed lanes) -- five
lanes in round 1, one in round 3, none of them the trouble. Switched off lane by lane, the solve's crossing room (each
lane's crossings spaced along it per layer, and the stack of crossers at one point) had to give on TX_D4 and DATA_CLK
in round 1 and on DATA_CLK alone in round 3 -- two pairs ending on the destination's facing face, every crossing of
theirs in the few millimetres between the arrays. The diagnosis solves the model again with that room free to break
lane by lane, the fewest lanes there can be, and names them (OUT.crowded.json; whole_route says them in the
round's log).

On a generated four-copper bus (16 lanes, shuffled, a 2.5 mm channel), its round-1 bench laid by one round of the
whole route, this pins:

1. as generated the solve has a plan, and the diagnosis on that model names no lane;
2. with a mover's crossings spaced as a stayer's (whole_solve.K_SWEEP 0) the solve finds no plan at all, and the
   diagnosis names lanes -- and without their nets the solve has a plan again. That solve of SOME of the bus's lanes
   is the last resort's partial, and was refused as not in the canonical frame: the remaining balls' centroids,
   source to destination, point a quarter off on this bench (asserted, so the check is not vacuous), and the guard
   (whole_ctx._guard) now takes the frame's own trunk, pad box to pad box, then;
3. whole_solve's main writes OUT.crowded.json for a model with no plan under SOLVE_CROWD=1, and nothing without it.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
LAYERS = 'F.Cu,B.Cu,In1.Cu,In2.Cu'

if os.environ.get('_CROWD_CHILD'):
    # (a child per check: the bench from the environment, whole_solve imported under it)
    sys.path.insert(0, AWX)
    sys.path.insert(1, os.path.join(ROOT, 'py_router'))
    os.chdir(AWX)
    import contextlib
    import io
    import whole_ctx
    import whole_solve as ws
    what = os.environ['_CROWD_CHILD']
    if os.environ.get('_CROWD_K_SWEEP'):
        ws.K_SWEEP = float(os.environ['_CROWD_K_SWEEP'])
    if what == 'main':
        sys.argv = ['whole_solve.py', sys.argv[1]]
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                ws.main()
            except SystemExit:
                pass
        sys.exit(0)
    refused = None
    J = None
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            ctx, _cs = whole_ctx.plan()
            J = ws.solve(ctx, os.environ['DEST'], crowd=(what == 'crowd'))
        except SystemExit as e_:
            refused = str(e_)          # (a stage's refusal of the bench: the finding, not a broken test)
    print(json.dumps({'plan': J is not None, 'any': ws.STATUS.get('plan'),
                      'crowded': (J or {}).get('crowded'), 'refused': refused}))
    sys.exit(0)


class _Kept:
    def __init__(self, d):
        self.d = d

    def __enter__(self):
        return self.d

    def __exit__(self, *a):
        return False


def env_of(**more):
    e = dict(os.environ)
    for k in ('_CROWD_CHILD', '_CROWD_K_SWEEP', 'SOLVE_CROWD', 'SOLVE_UNPROVED'):
        e.pop(k, None)
    e.update(ROUTE_LAYERS=LAYERS, DEST='SD1', OMP_NUM_THREADS='1', PLAN_PAGES='1', PLAN_JUDGE='ends',
             BRAID_PAIRS='1', PLAN_PAIRS='1', BRAID_EXACT_PAGES='0', PLAN_PAGES_SIDERS='2')
    e.update(more)
    return e


def child(what, bench, nets, k_sweep=None, args=(), **more):
    e = env_of(BENCH=bench, NETS=','.join(nets), _CROWD_CHILD=what, **more)
    if k_sweep is not None:
        e['_CROWD_K_SWEEP'] = str(k_sweep)
    r = subprocess.run([sys.executable, os.path.abspath(__file__)] + list(args), env=e, capture_output=True,
                       text=True, timeout=1800)
    if r.returncode != 0:
        raise SystemExit(f'BROKEN TEST: the {what} child died:\n{r.stderr[-2000:]}')
    return json.loads(r.stdout.strip().splitlines()[-1]) if what != 'main' else None


def main():
    print('=' * 60)
    print('the whole solve\'s CROWD diagnosis')
    print('=' * 60)
    fails = []
    keep = os.environ.get('CROWD_TEST_DIR')              # (a directory to work in and keep, to look at afterwards)
    if keep:
        os.makedirs(keep, exist_ok=True)
    with (tempfile.TemporaryDirectory() if not keep else _Kept(keep)) as td:
        raw, bench0 = os.path.join(td, 'raw.kicad_pcb'), os.path.join(td, 'bench.kicad_pcb')
        # (the bench and its round as a shell makes them, four routing layers throughout: whole_route sets its own
        # stage policy as it starts its stages)
        e = {k: v for k, v in os.environ.items() if not k.startswith('_CROWD')}
        e.update(ROUTE_LAYERS=LAYERS)
        for cmd in (['synth_bus.py', raw, '--k', '16', '--gap', '2.5', '--pattern', 'shuffle', '--seed', '1',
                     '--copper', '4'], ['make_bench.py', raw, 'SU1', 'SD1', bench0]):
            r = subprocess.run([sys.executable] + cmd, cwd=AWX, capture_output=True, text=True, timeout=600, env=e)
            if r.returncode != 0:
                raise SystemExit(f'BROKEN TEST: {cmd[0]} failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
        run = os.path.join(td, 'run')
        r = subprocess.run([sys.executable, 'whole_route.py', '16', run, '1'], cwd=AWX, capture_output=True,
                           text=True, timeout=1800, env=dict(e, BASE=bench0, DEST='SD1'))
        bench = os.path.join(run, 'r1', 'fo.kicad_pcb')
        if not os.path.isfile(bench):
            raise SystemExit(f'BROKEN TEST: one round laid no bench:\n{r.stdout[-2000:]}{r.stderr[-2000:]}')
        nets = [ln.strip() for ln in open(os.path.join(run, 'r1', 'nets.lines')) if ln.strip()]
        if len(nets) != 16:
            raise SystemExit(f'BROKEN TEST: the round ran {len(nets)} nets, not the 16 generated')
        # 1
        a = child('solve', bench, nets)
        c = child('crowd', bench, nets)
        if not a['plan']:
            raise SystemExit('BROKEN TEST: the generated bench has no plan as it is -- nothing to tighten')
        if c['crowded'] != []:
            fails.append(f'a model with a plan: the diagnosis named {c["crowded"]}, want none')
        # 2
        t = child('solve', bench, nets, k_sweep=0.0)
        if t['plan'] or t['any']:
            raise SystemExit('BROKEN TEST: movers spaced as stayers still leave a plan -- the bench does not '
                             'crowd, so the diagnosis is not tested')
        tc = child('crowd', bench, nets, k_sweep=0.0)
        named = tc['crowded'] or []
        if not named:
            fails.append('no plan for the crowded model: the diagnosis named no lane')
        else:
            rest = [n for n in nets if n.split('/')[-1] not in set(named)]
            if len(rest) != len(nets) - len(named):
                raise SystemExit(f'BROKEN TEST: the lanes named {named} are not the run\'s nets {nets}')
            sys.path.insert(0, AWX)
            sys.path.insert(1, os.path.join(ROOT, 'py_router'))
            import contextlib
            import io
            with contextlib.redirect_stdout(io.StringIO()):
                import flow_frame
                from kicad_parser import parse_kicad_pcb
                k_rest = flow_frame.quarter_of(parse_kicad_pcb(bench), 'SD1',
                                               {n.split('/')[-1] for n in rest})[0]
            print(f'  the lanes left: their balls\' centroids a quarter turn {k_rest} off')
            if not k_rest:
                raise SystemExit('BROKEN TEST: the lanes left are not a quarter off -- the guard is not tested')
            rs = child('solve', bench, rest, k_sweep=0.0)
            if not rs['plan']:
                fails.append(f'without the lanes named ({", ".join(named)}) the crowded model still has no plan'
                             + (f' -- refused: {rs["refused"].strip()[:160]}' if rs.get('refused') else ''))
        print(f'  crowded model: {len(named)} lane(s) named -- {", ".join(named)}')
        # 3
        for crowd_on in (True, False):
            out = os.path.join(td, f'main_{int(crowd_on)}.json')
            child('main', bench, nets, k_sweep=0.0, args=[out], SOLVE_UNPROVED='1',
                  **({'SOLVE_CROWD': '1'} if crowd_on else {}))
            cf = out[:-len('.json')] + '.crowded.json'
            if os.path.isfile(out):
                fails.append(f'main (SOLVE_CROWD {crowd_on}): wrote a plan for a model with none')
            if crowd_on and not (os.path.isfile(cf) and json.load(open(cf)).get('crowded')):
                fails.append('main under SOLVE_CROWD=1: no OUT.crowded.json naming lanes')
            if not crowd_on and os.path.isfile(cf):
                fails.append('main without SOLVE_CROWD: wrote OUT.crowded.json')
    for f in fails:
        print('  FAIL', f)
    print('PASS' if not fails else f'{len(fails)} FAILURE(S)')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
