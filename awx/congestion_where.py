#!/usr/bin/env python3
"""WHERE a refused lane is blocked: for each open net, the soft path
(negotiate.classify) and the point of every conflict with another
lane, binned by zone -- TOOTH (within `zone` mm of the source array's
pad box), BERTH (within `zone` of the destination's), CHANNEL (the rest)
-- per layer, plus the same for the blocker: which part of ITS lane
sits there (its escape / its ride / its landing).
usage: congestion_where.py FO BRAIDED K [--zone 1.0]"""
import argparse, json, os, sys, collections
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router')); sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb
import braid as te, negotiate as ng, surgical as sg, topo_strings as ts

ap = argparse.ArgumentParser()
ap.add_argument('fo'); ap.add_argument('braided'); ap.add_argument('k')
ap.add_argument('--zone', type=float, default=1.0)
ap.add_argument('--src', default='U1'); ap.add_argument('--dst', default='DU1')
a = ap.parse_args()
names = sg.k_nets(a.k).split(',')
ctx, groups = te.setup(a.fo, names, a.dst, lambda *x: None)
pcb_b = parse_kicad_pcb(a.braided)
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb_b.nets.items()}
nids = {byname[nm][0] for nm in names}
fo_c, cur_c, rest = ng.split_copper(ctx.pcb, pcb_b, nids)
fo = {nm: fo_c[byname[nm][0]] for nm in names}
state = {nm: cur_c[byname[nm][0]] for nm in names}
ends = {nm: (ctx.ends[nm][0], ctx.tooth_layer[nm], ctx.ends[nm][1], ctx.dest_layer[nm]) for nm in names}
N = ng.Negotiator(pcb_b, ctx.cfg, byname, ends, fo, state, rest,
                  window_pts={nm: [ctx.ends[nm][0], ctx.ends[nm][1]] for nm in names},
                  b_alts=ctx.dest_alts, chains=ctx.dest_chain)

def box(ref):
    fp = pcb_b.footprints[ref]
    xs = [p.global_x for p in fp.pads]; ys = [p.global_y for p in fp.pads]
    return (min(xs), min(ys), max(xs), max(ys))
SB, DB = box(a.src), box(a.dst)
def inbox(p, b, m):
    return b[0] - m <= p[0] <= b[2] + m and b[1] - m <= p[1] <= b[3] + m
def zone(p):
    if inbox(p, DB, a.zone): return 'BERTH'
    if inbox(p, SB, a.zone): return 'TOOTH'
    return 'CHANNEL'

rp = a.braided.rsplit('.', 1)[0] + '_refusals.json'
open_ = sorted(json.load(open(rp))) if os.path.exists(rp) else [nm for nm in names if not N.routed(nm)]
clr = ctx.cfg.clearance
tot = collections.Counter()
print(f'{os.path.basename(a.braided)}: open {open_}   src box {SB[0]:.1f}..{SB[2]:.1f} x {SB[1]:.1f}..{SB[3]:.1f}  dst box {DB[0]:.1f}..{DB[2]:.1f} x {DB[1]:.1f}..{DB[3]:.1f}')
for nm in open_:
    kind, path, blk = N.classify(nm)
    if kind != 'BLOCKED':
        print(f'  {nm}: {kind}'); continue
    psegs, pvias = path
    where = collections.Counter(); per = collections.defaultdict(collections.Counter)
    for s in psegs:
        A, B = (s.start_x, s.start_y), (s.end_x, s.end_y)
        for om in blk:
            osegs, ovias = N.lane(om)
            for o in osegs:
                if o.layer != s.layer: continue
                if ts.seg_seg_dist(A, B, (o.start_x, o.start_y), (o.end_x, o.end_y)) < (s.width + o.width) / 2 + clr - 1e-6:
                    mid = ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)
                    z = zone(mid); where[(z, s.layer[0])] += 1; per[om][z] += 1
            for v in ovias:
                if ts.seg_pt_dist(A, B, (v.x, v.y)) < v.size / 2 + s.width / 2 + clr - 1e-6:
                    z = zone((v.x, v.y)); where[(z, 'via')] += 1; per[om][z] += 1
    tot.update(where)
    (ax, ay), al, (bx, by), bl = ends[nm]
    print(f'  {nm:6s} tooth {al[0]} ({ax:.1f},{ay:.1f}) berth {bl[0]} ({bx:.1f},{by:.1f}) path {len(pvias)} via: '
          + ', '.join(f'{z}/{L} {c}' for (z, L), c in sorted(where.items(), key=lambda kv: -kv[1])))
    for om, zc in sorted(per.items(), key=lambda kv: -sum(kv[1].values())):
        (ox, oy), ol, (px, py), pl = ends[om]
        print(f'      {om:6s} {dict(zc)}   its tooth {ol[0]} ({ox:.1f},{oy:.1f}) berth {pl[0]} ({px:.1f},{py:.1f})')
print('TOTAL', ', '.join(f'{z}/{L} {c}' for (z, L), c in sorted(tot.items(), key=lambda kv: -kv[1])))
