#!/usr/bin/env python3
"""Two lanes the whole route left open on generated four-copper buses, each by a defect of its own.

  python3 tests/test_622_lane_fixes.py

1. A net's SOURCE part was the part of its pad nearest the tooth's stub end (braid.setup's src_ref). A dog-bone run
   down the channel ends nearer a ball of the DESTINATION than any of its own, so the net read as starting at the
   destination, and the whole route -- which plans the lanes of the run's one source -- left it out of every frame:
   open, every round (generated c4s14g25, SYN07). The destination's own pads are no source now. Pinned: after one
   round, every net of the run is a lane of the frame; and at least one tooth's stub ends nearer a ball of the
   destination than a ball of its source, or the bench does not exercise the defect (BROKEN TEST).
2. A crossed pair's span between two poses that nearly coincide -- the plan put its crossover AT its berth, the
   crossover's exit pose 0.075 mm from the berth's -- went to the pair router, which refuses two poses that close
   ("no pair corridor exists"): the pair was refused in its band, and laid free of the plan or not at all (generated
   c4p16g25, SYP1). Such a span is joined leg to leg now (connect._connect_pair_crossover). Pinned: every crossed pair
   of the plan the round laid with its crossover's entry or exit within a pair's width of the end's handover is laid IN
   ITS BAND; and there is one, or the bench does not exercise the defect (BROKEN TEST).

Each case generated, its bench made, and one round of whole_route.py run (four routing layers), about a minute each.
The round's grade line is printed, not asserted: another machine may lay other copper.
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
LAYERS = 'F.Cu,B.Cu,In1.Cu,In2.Cu'

if os.environ.get('_LANE_FIXES_CHILD') == 'frame':
    # (check 1's child: the frame of the round's bench, as every whole_* stage reads it)
    sys.path.insert(0, AWX)
    sys.path.insert(1, os.path.join(ROOT, 'py_router'))
    os.chdir(AWX)
    import contextlib
    import io
    import whole_ctx
    import whole_frame
    with contextlib.redirect_stdout(io.StringIO()):
        ctx, _cs = whole_ctx.plan()
        Fr = whole_frame.build(ctx, 'SD1')
    legs = {l_ for pr in (getattr(ctx, 'pairs', {}) or {}).values() for l_ in pr}
    lanes = set(Fr.M)
    # (the bench exercises the defect where a tooth's stub ends nearer a ball of the destination than of its source)
    near_dest = []
    for n in ctx.ends:
        nid, net = ctx.byname[n]
        t_ = ctx.ends[n][0]
        d_dst = min((math.hypot(p.global_x - t_[0], p.global_y - t_[1]) for p in net.pads if p.component_ref == 'SD1'),
                    default=math.inf)
        d_src = min((math.hypot(p.global_x - t_[0], p.global_y - t_[1]) for p in net.pads if p.component_ref == 'SU1'),
                    default=math.inf)
        if d_dst < d_src:
            near_dest.append(n)
    nets = [n.split('/')[-1] for n in os.environ['NETS'].split(',') if n]
    print(json.dumps({'missing': sorted(n for n in nets if n not in lanes and n not in legs),
                      'near_dest': sorted(near_dest), 'src': {n: ctx.src_ref[n] for n in ctx.src_ref}}))
    sys.exit(0)


def case(td, tag, k, args):
    """the generated case's round-1 directory, after one round of whole_route.py"""
    d = os.path.join(td, tag)
    os.makedirs(d)
    raw, bench = os.path.join(d, 'raw.kicad_pcb'), os.path.join(d, 'bench.kicad_pcb')
    e = {k_: v_ for k_, v_ in os.environ.items() if not k_.startswith('_LANE')}
    e.update(ROUTE_LAYERS=LAYERS)
    for cmd in (['synth_bus.py', raw, '--k', str(k)] + args, ['make_bench.py', raw, 'SU1', 'SD1', bench]):
        r = subprocess.run([sys.executable] + cmd, cwd=AWX, capture_output=True, text=True, timeout=600, env=e)
        if r.returncode != 0:
            raise SystemExit(f'BROKEN TEST: {tag}: {cmd[0]} failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
    run = os.path.join(d, 'run')
    r = subprocess.run([sys.executable, 'whole_route.py', str(k), run, '1'], cwd=AWX, capture_output=True, text=True,
                       timeout=1800, env=dict(e, BASE=bench, DEST='SD1'))
    if not os.path.isfile(os.path.join(run, 'r1', 'fo.kicad_pcb')):
        raise SystemExit(f'BROKEN TEST: {tag}: one round laid no bench:\n{r.stdout[-2000:]}{r.stderr[-2000:]}')
    whole = [ln for ln in r.stdout.splitlines() if ln.startswith('WHOLE')]
    print(f'  {tag}: {whole[-1] if whole else "no grade line"}')
    return os.path.join(run, 'r1')


def main():
    print('=' * 60)
    print('lanes the whole route left open: the source part, a crossover at its berth')
    print('=' * 60)
    fails = []
    with tempfile.TemporaryDirectory() as td:
        # 1
        r1 = case(td, 'c4s14g25', 14, ['--gap', '2.5', '--pattern', 'shuffle', '--seed', '1', '--copper', '4'])
        nets = [ln.strip() for ln in open(os.path.join(r1, 'nets.lines')) if ln.strip()]
        e = {k_: v_ for k_, v_ in os.environ.items() if not k_.startswith('_LANE')}
        e.update(_LANE_FIXES_CHILD='frame', BENCH=os.path.join(r1, 'fo.kicad_pcb'), NETS=','.join(nets),
                 DEST='SD1', ROUTE_LAYERS=LAYERS, PLAN_PAGES='1', PLAN_JUDGE='ends', BRAID_PAIRS='1', PLAN_PAIRS='1')
        r = subprocess.run([sys.executable, os.path.abspath(__file__)], env=e, capture_output=True, text=True,
                           timeout=900)
        if r.returncode != 0:
            raise SystemExit(f'BROKEN TEST: the frame child died:\n{r.stderr[-2000:]}')
        got = json.loads(r.stdout.strip().splitlines()[-1])
        if not got['near_dest']:
            raise SystemExit('BROKEN TEST: no tooth of c4s14g25 ends nearer a ball of the destination -- the bench does '
                             'not exercise the source part')
        print(f"  teeth ending nearer the destination's balls: {got['near_dest']} "
              f"(their source read as {[got['src'].get(n) for n in got['near_dest']]})")
        if got['missing']:
            fails.append(f"c4s14g25: nets left out of the frame: {got['missing']} (source read as "
                         f"{[got['src'].get(n) for n in got['missing']]})")
        # 2
        r2 = case(td, 'c4p16g25', 16, ['--gap', '2.5', '--pattern', 'shuffle', '--seed', '3', '--pairs', '6',
                                       '--copper', '4'])
        loopd = os.path.join(r2, 'loop')
        plan_f = next((os.path.join(loopd, f) for f in ('plan.json', 'held_plan.json')
                       if os.path.isfile(os.path.join(loopd, f))), None)
        if plan_f is None:
            raise SystemExit('BROKEN TEST: c4p16g25: the round laid no plan')
        plan = json.load(open(plan_f))
        close = []
        for n, L in plan['lanes'].items():
            x_, ends_ = L.get('cross'), L.get('ends')
            if not x_ or not ends_ or len(ends_) != 2:
                continue
            mid = lambda pn: ((pn['P'][0] + pn['N'][0]) / 2, (pn['P'][1] + pn['N'][1]) / 2)
            hand = lambda e_: ((e_['handover'][0][0] + e_['handover'][1][0]) / 2,
                               (e_['handover'][0][1] + e_['handover'][1][1]) / 2)
            width = math.hypot(x_['entry']['P'][0] - x_['entry']['N'][0], x_['entry']['P'][1] - x_['entry']['N'][1])
            for pose, e_ in ((mid(x_['entry']), ends_[0]), (mid(x_['exit']), ends_[1])):
                if math.hypot(pose[0] - hand(e_)[0], pose[1] - hand(e_)[1]) < width:
                    close.append(n)
        if not close:
            raise SystemExit('BROKEN TEST: c4p16g25: no crossed pair of the plan has its crossover at an end -- the '
                             'bench does not exercise the short span')
        print(f'  crossed pairs with their crossover at an end: {sorted(set(close))}')
        seq_log = open(os.path.join(r2, 'route_seq.log')).read()
        how = {m.group(1): m.group(2) for m in re.finditer(r'^(\S+)\s+pair (IN BAND|FREE|REFUSED)', seq_log, re.M)}
        off = {n: how.get(n, 'not laid') for n in sorted(set(close)) if how.get(n) != 'IN BAND'}
        if off:
            fails.append(f'c4p16g25: crossed pairs with their crossover at an end not laid in their bands: {off}')
    for f in fails:
        print('  FAIL', f)
    print('PASS' if not fails else f'{len(fails)} FAILURE(S)')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
