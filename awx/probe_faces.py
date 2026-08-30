#!/usr/bin/env python3
"""Which FACE of each array does every net's free end sit on, in the
flow frame? The corridor-per-face-pair refactor groups nets by
(source face, destination face), so the first thing to know is which
pairs the bench actually populates at each K -- on the bare bench (the
source is fanned out, the destination is not: destination = the ball)
and on a chained fanout board (both ends free stub ends).

usage: probe_faces.py BOARD K [--dest REF]
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import flow_frame as ff  # noqa: E402
import braid as te  # noqa: E402


def face_of(pt, bbox, margin=0.1):
    """E/W/N/S: the side of the (padded) bbox the point lies beyond;
    'in' if inside."""
    x0, y0, x1, y1 = bbox
    dx = (pt[0] - x1) if pt[0] > x1 else (x0 - pt[0]) if pt[0] < x0 else 0
    dy = (pt[1] - y1) if pt[1] > y1 else (y0 - pt[1]) if pt[1] < y0 else 0
    if dx <= margin and dy <= margin:
        return 'in'
    if dx >= dy:
        return 'E' if pt[0] > x1 else 'W'
    return 'S' if pt[1] > y1 else 'N'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--dest', default=None)
    a = ap.parse_args()
    nets = subprocess.run([sys.executable,
                           os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True,
                          text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    probe = te.endpoints(pcb, names, byname, dest_ref=a.dest)
    theta = ff.flow_angle([probe[n][0] for n in probe],
                          [probe[n][1] for n in probe])
    cx = sum(probe[n][1][0] for n in probe) / len(probe)
    cy = sum(probe[n][1][1] for n in probe) / len(probe)
    pcb, back = ff.rotate_pcb(pcb, theta, cx, cy)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    ends = te.endpoints(pcb, names, byname, dest_ref=a.dest)
    dest_ref = ends[names[0]][2]
    src_refs = {}
    for nm in names:
        nid, net = byname[nm]
        p = min(net.pads, key=lambda q: (q.global_x - ends[nm][0][0]) ** 2
                + (q.global_y - ends[nm][0][1]) ** 2)
        src_refs[nm] = p.component_ref

    def bbox(ref):
        ps = pcb.footprints[ref].pads
        return (min(p.global_x for p in ps), min(p.global_y for p in ps),
                max(p.global_x for p in ps), max(p.global_y for p in ps))
    sref = max(set(src_refs.values()), key=list(src_refs.values()).count)
    sb, db = bbox(sref), bbox(dest_ref)
    print(f'flow theta={theta:.0f}  source {sref} bbox '
          f'({sb[0]:.2f},{sb[1]:.2f})-({sb[2]:.2f},{sb[3]:.2f})  '
          f'dest {dest_ref} bbox ({db[0]:.2f},{db[1]:.2f})-'
          f'({db[2]:.2f},{db[3]:.2f})')
    pairs = {}
    for nm in names:
        sf = face_of(ends[nm][0], sb)
        df = face_of(ends[nm][1], db)
        pairs.setdefault((sf, df), []).append(nm)
    for (sf, df), ms in sorted(pairs.items()):
        print(f'  {sf}->{df} ({len(ms)}): {" ".join(ms)}')
    print(f'{"net":7} {"src":>16} sf {"dst":>16} df')
    for nm in sorted(names, key=lambda n: ends[n][0][1]):
        s, t = ends[nm][0], ends[nm][1]
        print(f'{nm:7} ({s[0]:7.2f},{s[1]:6.2f}) {face_of(s, sb):2} '
              f'({t[0]:7.2f},{t[1]:6.2f}) {face_of(t, db):2}')


if __name__ == '__main__':
    main()
