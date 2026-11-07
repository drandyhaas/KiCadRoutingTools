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
   c4p16g25, SYP1). Such a span is joined leg to leg now (connect._connect_pair_crossover). The plan seldom crosses a
   generated bench's pairs any more (that bench none; of 40 generated benches one, its crossover 0.3 mm from an end),
   while a bus whose berths are a connector's pads has crossovers 0.1 mm from an end; so the span is built: a
   crossover (pairs.crossover) in the channel of case 1's generated board, given an end connector 0.075 mm past each
   of its poses, laid by connect.connect_pair. Pinned: both spans laid, each joined P to P and N to N on its pose's
   layer; and the crossover fits there, or the check tests nothing (BROKEN TEST).

Case 1 generated, its bench made, and one round of whole_route.py run (four routing layers), about a minute; case 2
laid on that board in seconds. The round's grade line is printed, not asserted: another machine may lay other copper.
"""
import json
import math
import os
import subprocess
import sys
import tempfile

try:
    import ortools  # noqa: F401  (the whole route's solve is a CP-SAT solve)
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

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

if os.environ.get('_LANE_FIXES_CHILD') == 'cross':
    # (check 2's child: a crossover in the channel of BOARD, an end connector given AT each of its poses, laid by
    # connect_pair as the pair step lays a plan's crossed pair)
    sys.path.insert(0, AWX)
    sys.path.insert(1, os.path.join(ROOT, 'py_router'))
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        import braid as bd
        import connect as cn
        import pairs as pr
        from kicad_parser import Segment, parse_kicad_pcb
        pcb = parse_kicad_pcb(os.environ['BOARD'])
    cfg = bd.run_config(pcb, os.environ['BOARD'])
    su, sd, g = pcb.footprints['SU1'], pcb.footprints['SD1'], cfg.grid_step
    V = (round((su.x + sd.x) / 2 / g) * g, round((su.y + sd.y) / 2 / g) * g)          # the channel's middle
    u = (0.0, 1.0) if abs(sd.x - su.x) > abs(sd.y - su.y) else (1.0, 0.0)            # along it
    p_id = max(pcb.nets) + 1
    n_id = p_id + 1
    half = pr.pitch(cfg.track_width) / 2
    c = pr.crossover(V, u, 1, half, cfg.via_size, pr.dive_offset(cfg, half), cfg.track_width, cfg.clearance, g,
                     'F.Cu', 'B.Cu', p_id, n_id, first='P', floor=pr.handover_setback(cfg))
    if c is None:
        print(json.dumps({'crossover': None}))
        sys.exit(0)
    c.update(first='P', at=list(V), heading=list(u), layers=['F.Cu', 'B.Cu'])     # (as whole_snap records it)
    GAP_MM = 0.075
    joins = []

    def given(pose, d, L):
        """an end connector whose handover stands d along u from the crossover's pose (its P, N), its legs on 0.5 mm
        to the end's tips: (given, tip P, tip N)"""
        hp, hn = [(q[0] + u[0] * d, q[1] + u[1] * d) for q in (pose['P'], pose['N'])]
        s = 0.5 if d > 0 else -0.5
        tp, tn = [(q[0] + u[0] * s, q[1] + u[1] * s) for q in (hp, hn)]
        joins.extend([(tuple(pose['P']), hp, L, p_id), (tuple(pose['N']), hn, L, n_id)])
        legs = [Segment(hp[0], hp[1], tp[0], tp[1], cfg.track_width, L, p_id),
                Segment(hn[0], hn[1], tn[0], tn[1], cfg.track_width, L, n_id)]
        return (legs, [], hp, hn, u if d < 0 else (-u[0], -u[1]), L), tp, tn
    ag, ap, an = given(c['entry'], -GAP_MM, 'F.Cu')
    bg, bp, bn = given(c['exit'], GAP_MM, 'B.Cu')
    rep = {}
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        res = cn.connect_pair(pcb, p_id, n_id, ap, an, 'F.Cu', bp, bn, 'B.Cu', cfg, report=rep, gap=pr.GAP,
                              a_dir=(-u[0], -u[1]), b_dir=u, a_given=ag, b_given=bg, x_given=c)
    near = lambda a_, b_: math.hypot(a_[0] - b_[0], a_[1] - b_[1]) < 1e-6

    def has(a_, b_, L, nid):
        """a segment of net nid on L from a_ to b_ (either way) among the laid copper"""
        for s in res[0]:
            p_, q_ = (s.start_x, s.start_y), (s.end_x, s.end_y)
            if s.net_id == nid and s.layer == L and (near(p_, a_) and near(q_, b_) or near(p_, b_) and near(q_, a_)):
                return True
        return False
    found = [res is not None and has(*j) for j in joins]
    why = [ln.strip() for ln in buf.getvalue().splitlines() if 'corridor' in ln or 'stuck' in ln][:2]
    print(json.dumps({'crossover': [round(x, 3) for x in c['span']], 'laid': res is not None, 'joined': found,
                      'cross_span': rep.get('cross_span'), 'why': why}))
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
        board = os.path.join(td, 'c4s14g25', 'raw.kicad_pcb')
        e2 = {k_: v_ for k_, v_ in os.environ.items() if not k_.startswith('_LANE')}
        e2.update(_LANE_FIXES_CHILD='cross', BOARD=board, ROUTE_LAYERS=LAYERS)
        r = subprocess.run([sys.executable, os.path.abspath(__file__)], env=e2, capture_output=True, text=True,
                           timeout=600)
        if r.returncode != 0:
            raise SystemExit(f'BROKEN TEST: the crossover child died:\n{r.stderr[-2000:]}')
        got = json.loads(r.stdout.strip().splitlines()[-1])
        if got['crossover'] is None:
            raise SystemExit('BROKEN TEST: no crossover fits in the channel of c4s14g25 -- the check tests nothing')
        print(f"  a crossover in the channel (span {got['crossover']}), each end's handover 0.075 mm past its pose: "
              f"laid {got['laid']}, joined {got['joined']}")
        if not got['laid'] or not all(got['joined']):
            fails.append(f"a crossover at both its ends: laid {got['laid']}, joined P/N at entry and exit "
                         f"{got['joined']}, refused in span {got['cross_span']} {got['why']}")
    for f in fails:
        print('  FAIL', f)
    print('PASS' if not fails else f'{len(fails)} FAILURE(S)')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
