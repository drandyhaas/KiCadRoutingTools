#!/usr/bin/env python3
"""Route a corridor exactly as the braid's attempt 0 does, then print
every routed lane's copper in the spine's (s, o) frame as runs -- where
it actually went against where the plan put it -- and every refused
lane's plan. usage: probe_copper.py FANOUT_BOARD K [--dest REF]
[--corridor N] [--s-from A --s-to B] [--nets N1,N2]"""
import argparse
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import braid as te  # noqa: E402
import connect as cn  # noqa: E402
from schedule import Schedule  # noqa: E402


def runs_of(c, segs, vias, s_from, s_to):
    """Compress a lane's copper into (s_a, s_b, layer, o_min, o_max)."""
    sp = c.spine
    pts = []
    for sg in segs:
        n = max(2, int(math.hypot(sg.end_x - sg.start_x, sg.end_y - sg.start_y) / 0.05) + 1)
        for t in np.linspace(0, 1, n):
            x = sg.start_x + t * (sg.end_x - sg.start_x)
            y = sg.start_y + t * (sg.end_y - sg.start_y)
            s, o = sp.project_pt((x, y))
            if s_from <= s <= s_to:
                pts.append((s, o, sg.layer[0]))
    pts.sort()
    out = []
    for (s, o, L) in pts:
        if out and out[-1][2] == L and s - out[-1][1] < 0.2:
            a, b, _L, lo, hi = out[-1]
            out[-1] = (a, s, L, min(lo, o), max(hi, o))
        else:
            out.append((s, s, L, o, o))
    vs = []
    for v in vias:
        s, o = sp.project_pt((v.x, v.y))
        if s_from <= s <= s_to:
            vs.append((s, o))
    return out, vs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--corridor', type=int, default=0)
    ap.add_argument('--s-from', type=float, default=-1e9)
    ap.add_argument('--s-to', type=float, default=1e9)
    ap.add_argument('--nets', default='')
    ap.add_argument('--box', default='',
                    help='s_a,s_b,o_a,o_b: list every copper item inside')
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    quiet = lambda *x: None  # noqa: E731
    ctx, groups = te.setup(a.board, names, a.dest, quiet)
    grp = groups[a.corridor]
    c = te.Corridor(a.corridor, grp, ctx, quiet)
    c.build_spine()
    c.classify()
    c.offsets(0.35)
    c.reserve_intervals()
    sched = Schedule(c.launch, c.target, ctx.tooth_layer,
                     dest_layer=ctx.dest_layer)
    cols = sched.columns({d: 1 for d in sched.divers}, {d: 0 for d in sched.divers})
    c.lay_lanes(cols)
    order = list(sched.priority) + [x for x in c.target if x not in sched.divers]
    routed = {}
    refused = []
    for nm in order:
        virt = c.virtual_of([x for x in grp if x != nm and x not in routed])
        res = c.route_lane(nm, virt)
        if res is None:
            refused.append(nm)
            continue
        routed[nm] = res
    show = [n for n in a.nets.split(',') if n] or grp
    print(f'order: {order}')
    print(f'refused: {refused}')
    for nm in show:
        s_t, o_t = c.st[nm]
        s_e, o_e = c.se[nm]
        plan = ' '.join(f'({s:.1f},{o:.2f})' for s, o in c.mid[nm]
                        if a.s_from - 1 <= s <= a.s_to + 1)
        print(f'\n{nm}: tooth ({s_t:.2f},{o_t:.2f}) {ctx.tooth_layer[nm][0]}'
              f'  stub ({s_e:.2f},{o_e:.2f}) {ctx.dest_layer[nm][0]}'
              f'  launch {c.launch_o[nm]:.2f} target {c.target_o[nm]:.2f}'
              f'  {"ROUTED" if nm in routed else "REFUSED"}')
        print(f'  plan: {plan}')
        print(f'  legs: ' + ' '.join(f'(s{s:.2f} o{oa:.2f}->{ob:.2f})' for s, oa, ob in c.legs[nm])
              + f'  req: {[(round(x, 2), round(y, 2), L[0]) for x, y, L in c.req[nm]]}'
              + f'  bwin: {[(round(max(x, -99), 2), round(min(y, 99), 2)) for x, y in c.bwin[nm]]}')
        if nm in routed:
            segs, vias = routed[nm]
            rr, vs = runs_of(c, segs, vias, a.s_from, a.s_to)
            print('  copper: ' + ' '.join(
                f'[{s_a:.1f}..{s_b:.1f} {L} o {lo:.2f}..{hi:.2f}]' for s_a, s_b, L, lo, hi in rr))
            if vs:
                print('  vias: ' + ' '.join(f'({s:.2f},{o:.2f})' for s, o in vs))
    if a.box:
        s_a, s_b, o_a, o_b = [float(v) for v in a.box.split(',')]
        sp = c.spine
        id2name = {i: n.name.split('/')[-1] for i, n in ctx.pcb.nets.items()}
        print(f'\ncopper in s [{s_a},{s_b}] o [{o_a},{o_b}]:')
        for sg in ctx.pcb.segments:
            n = max(2, int(math.hypot(sg.end_x - sg.start_x, sg.end_y - sg.start_y) / 0.05) + 1)
            inside = []
            for t in np.linspace(0, 1, n):
                x = sg.start_x + t * (sg.end_x - sg.start_x)
                y = sg.start_y + t * (sg.end_y - sg.start_y)
                s, o = sp.project_pt((x, y))
                if s_a <= s <= s_b and o_a <= o <= o_b:
                    inside.append((s, o))
            if inside:
                print(f'  seg {id2name.get(sg.net_id, sg.net_id):8} {sg.layer[0]} '
                      f'w{sg.width:.3f} ({inside[0][0]:.2f},{inside[0][1]:.2f})'
                      f'->({inside[-1][0]:.2f},{inside[-1][1]:.2f})')
        for v in ctx.pcb.vias:
            s, o = sp.project_pt((v.x, v.y))
            if s_a <= s <= s_b and o_a <= o <= o_b:
                print(f'  via {id2name.get(v.net_id, v.net_id):8} ({s:.2f},{o:.2f}) d{v.size}')
        for ref, fp in ctx.pcb.footprints.items():
            for p in fp.pads:
                s, o = sp.project_pt((p.global_x, p.global_y))
                if s_a <= s <= s_b and o_a <= o <= o_b:
                    print(f'  pad {ref}.{p.pad_number} {id2name.get(p.net_id, p.net_id)} '
                          f'({s:.2f},{o:.2f}) {p.size_x:.2f}x{p.size_y:.2f} {p.layers}')


if __name__ == '__main__':
    main()
