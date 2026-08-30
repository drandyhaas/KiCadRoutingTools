#!/usr/bin/env python3
"""Show the corridor clustering's decisions for a fanout board at K:
every stub pair within the linkage distance with its two rung tests,
the groups, and the reach check per member. usage: probe_cluster.py
BOARD K [--dest REF] [--pair A,B]"""
import argparse
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402
import corridor as cr  # noqa: E402
import detect_buses as db  # noqa: E402
import topo_strings as ts  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--pair', default=None)
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    kids = {byname[nm][0] for nm in names}
    ends = te.endpoints(pcb, names, byname, dest_ref=a.dest)
    cache = {}

    def obs(nm):
        if nm not in cache:
            cache[nm] = te.build_obstacles(pcb, byname[nm][0], kids, 'F.Cu')
        return cache[nm]
    paths = db.taut_paths(names, ends, obs)
    src_refs = set()
    for nm in names:
        nid, net = byname[nm]
        src_refs.add(min(net.pads, key=lambda p: ts.d2(
            (p.global_x, p.global_y), ends[nm][0])).component_ref)
    pad_obs = te.array_pad_obstacles(pcb, src_refs | {a.dest})
    stubs = {nm: ends[nm][1] for nm in names}
    teeth = {nm: ends[nm][0] for nm in names}
    appr = {nm: cr.point_before_end(paths[nm], 2.0) for nm in names}
    pairs = [tuple(a.pair.split(','))] if a.pair else [
        (x, y) for i, x in enumerate(names) for y in names[i + 1:]]
    for (x, y) in pairs:
        d = math.hypot(stubs[x][0] - stubs[y][0], stubs[x][1] - stubs[y][1])
        if d > 6.0 and not a.pair:
            continue
        oa = pad_obs.seg_clear(appr[x], appr[y])
        os_ = pad_obs.seg_clear(stubs[x], stubs[y])
        if not (oa or os_) or a.pair:
            print(f'{x}-{y}: {d:.2f} mm  approach {appr[x]}->{appr[y]} '
                  f'{"clear" if oa else "BLOCKED " + str(pad_obs.hugs([appr[x], appr[y]]))}  '
                  f'stub rung {"clear" if os_ else "BLOCKED"}')
    def centre_of(ref):
        ps = pcb.footprints[ref].pads
        return (sum(p.global_x for p in ps) / len(ps),
                sum(p.global_y for p in ps) / len(ps))
    groups = cr.cluster_corridors(
        names, paths, teeth, stubs, pad_obs.seg_clear, D=6.0, log=print,
        dest_ref={nm: ends[nm][2] for nm in names},
        centres={nm: centre_of(ends[nm][2]) for nm in names})
    print('groups:', groups)


if __name__ == '__main__':
    main()
