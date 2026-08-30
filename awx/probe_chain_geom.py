#!/usr/bin/env python3
"""Where does the chained braid's corridor actually lie?

The K15 chain printed `column pitch -0.030`: a NEGATIVE corridor width.
x1 is derived from the destination's ball field (fx0 - 0.8*pitch), but
in --dest-stubs mode the braid delivers to the STUB ENDS, which the
fanout laid west of the field -- so the field-derived x1 can sit west
of the source teeth's x0. Print every quantity the trunk geometry is
built from, in the flow frame, so the fix is aimed at a measured gap
and not a guessed one.

usage: probe_chain_geom.py BOARD K [--dest-stubs REF]
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import flow_frame as ff  # noqa: E402
import topo_emit as te  # noqa: E402
import subprocess  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--dest-stubs', default=None)
    ap.add_argument('--pitch', type=float, default=0.8)
    a = ap.parse_args()
    nets = subprocess.run([sys.executable,
                           os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True,
                          text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    probe = te.endpoints(pcb, names, byname, dest_ref=a.dest_stubs)
    theta = ff.flow_angle([probe[n][0] for n in probe],
                          [probe[n][1] for n in probe])
    cx = sum(probe[n][1][0] for n in probe) / len(probe)
    cy = sum(probe[n][1][1] for n in probe) / len(probe)
    pcb, back = ff.rotate_pcb(pcb, theta, cx, cy)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    ends = te.endpoints(pcb, names, byname, dest_ref=a.dest_stubs)
    print(f'flow frame theta={theta:.0f}')
    comps = {ends[nm][2] for nm in names}
    for c in sorted(comps):
        xs = [p.global_x for p in pcb.footprints[c].pads]
        ys = [p.global_y for p in pcb.footprints[c].pads]
        print(f'{c}: pads x [{min(xs):.2f},{max(xs):.2f}] '
              f'y [{min(ys):.2f},{max(ys):.2f}]')
    srcs = {}
    for nm in names:
        # which component owns the source end
        nid, net = byname[nm]
        p = min(net.pads, key=lambda q: (q.global_x - ends[nm][0][0]) ** 2
                + (q.global_y - ends[nm][0][1]) ** 2)
        srcs[nm] = p.component_ref
    for c in sorted(set(srcs.values())):
        xs = [p.global_x for p in pcb.footprints[c].pads]
        ys = [p.global_y for p in pcb.footprints[c].pads]
        print(f'{c} (source): pads x [{min(xs):.2f},{max(xs):.2f}] '
              f'y [{min(ys):.2f},{max(ys):.2f}]')
    fx0 = min(p.global_x for c in comps for p in pcb.footprints[c].pads)
    x1 = fx0 - a.pitch * 0.8
    x0 = max(ends[nm][0][0] for nm in names) + 0.3
    print(f'fx0={fx0:.2f}  x1(field)={x1:.2f}  x0(teeth)={x0:.2f}  '
          f'x1-x0={x1 - x0:.2f}')
    tx = [ends[nm][1][0] for nm in names]
    print(f'stub-end x: [{min(tx):.2f},{max(tx):.2f}]  '
          f'x1(stubs)={min(tx) - 0.3:.2f}  '
          f'corridor={min(tx) - 0.3 - x0:.2f}')

    def layer_at(nid, pt):
        return next((s.layer for s in pcb.segments if s.net_id == nid
                     and (abs(s.start_x - pt[0]) + abs(s.start_y - pt[1])
                          < 0.005 or abs(s.end_x - pt[0])
                          + abs(s.end_y - pt[1]) < 0.005)), '?')
    print(f'{"net":7} {"src":>16} {"L":4} {"dst":>16} {"L":4}')
    for nm in sorted(names, key=lambda n: ends[n][0][1]):
        nid = byname[nm][0]
        s, t = ends[nm][0], ends[nm][1]
        print(f'{nm:7} ({s[0]:7.2f},{s[1]:6.2f}) {layer_at(nid, s):4} '
              f'({t[0]:7.2f},{t[1]:6.2f}) {layer_at(nid, t):4}')


if __name__ == '__main__':
    main()
