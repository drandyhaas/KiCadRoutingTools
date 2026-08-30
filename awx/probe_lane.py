#!/usr/bin/env python3
"""Route ONE lane of a corridor under varying constraints, to see what
refuses it: the plan alone (band + virtual copper, as the braid does),
the band without the virtual copper, and no constraint at all (the
static copper and the window only). The other lanes are NOT routed
first, so this is the lane's own feasibility against the plan.

usage: probe_lane.py FANOUT_BOARD K NET [--dest REF]
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import braid as te  # noqa: E402
import connect as cn  # noqa: E402
from schedule import Schedule  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('net')
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--after', action='store_true',
                    help='route the other lanes first, in the braid\'s '
                         'order, so the probe sees the copper the braid '
                         'saw when it refused this one')
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    ctx, groups = te.setup(a.board, names, a.dest, print)
    grp = next(g for g in groups if a.net in g)
    c = te.Corridor(0, grp, ctx, print)
    c.build_spine()
    c.classify()
    c.offsets(0.35)
    c.reserve_intervals()
    sched = Schedule(c.launch, c.target, ctx.tooth_layer, log=print)
    cols, gate = c.plan_columns(sched, {d: 1 for d in sched.divers},
                                {d: 0 for d in sched.divers})
    print(f'  {len(cols)} columns, gate {gate}')
    c.lay_lanes(cols)
    nm = a.net
    c.debug_lane(nm)
    nid = ctx.byname[nm][0]
    others = [om for om in grp if om != nm]
    if a.after:
        order = list(sched.priority) + [x for x in c.target if x not in sched.divers]
        routed = set()
        for om in order:
            if om == nm:
                break
            unrouted = [x for x in grp if x != om and x not in routed]
            # exactly the braid's call (route_lane appends the copper)
            res = c.route_lane(om, c.virtual_of(unrouted), c.virtual_vias_of(unrouted))
            if res is None:
                print(f'  (earlier lane {om} refused)')
                continue
            routed.add(om)
            print(f'  routed {om}: {len(res[1])} via(s) at '
                  + ' '.join(f'({v.x:.2f},{v.y:.2f})' for v in res[1]))
        others = [om for om in grp if om != nm and om not in routed]
    os.environ['CONNECT_DEBUG'] = '1'
    vv = c.virtual_vias_of(others)
    for label, band, virt, vvs in (
            ('plan (band + virtual)', c.band_of(nm), c.virtual_of(others), vv),
            ('band only', c.band_of(nm), None, None),
            ('virtual only', None, c.virtual_of(others), vv),
            ('free', None, None, None)):
        res = cn.connect(ctx.pcb, nid, c.teeth[nm], ctx.tooth_layer[nm],
                         c.stubs[nm], ctx.dest_layer[nm], ctx.cfg, band=band,
                         virtual=virt, margin=0.6, window_pts=c.lane_xy[nm],
                         virtual_vias=vvs)
        if res is None:
            print(f'{label:24}: REFUSED')
        else:
            segs, vias = res
            print(f'{label:24}: routed, {len(segs)} segments, {len(vias)} via(s) at '
                  + ' '.join(f'({v.x:.2f},{v.y:.2f})' for v in vias))


if __name__ == '__main__':
    main()
