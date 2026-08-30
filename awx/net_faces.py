#!/usr/bin/env python3
"""For each net: which FACE of each array its copper leaves through, its
vias (inside an array's outline or in the open), and the layer of its
longest run -- on any routed board (the human original, or ours).
usage: net_faces.py BOARD NET [NET ...] [--refs U1,DU1]"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402


def bbox(fp, pad=0.45):
    xs = [p.global_x for p in fp.pads]
    ys = [p.global_y for p in fp.pads]
    return (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)


def inside(p, b):
    return b[0] <= p[0] <= b[2] and b[1] <= p[1] <= b[3]


def face(p, b):
    d = {'left': p[0] - b[0], 'right': b[2] - p[0],
         'up': p[1] - b[1], 'down': b[3] - p[1]}
    return min(d, key=d.get)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('nets', nargs='+')
    ap.add_argument('--refs', default='U1,DU1')
    a = ap.parse_args()
    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}
    boxes = {r: bbox(pcb.footprints[r]) for r in a.refs.split(',')}
    for nm in a.nets:
        nid = byname.get(nm)
        if nid is None:
            print(f'{nm}: not on board')
            continue
        segs = [s for s in pcb.segments if s.net_id == nid]
        vias = [v for v in pcb.vias if v.net_id == nid]
        out = [nm + ':']
        for ref, b in boxes.items():
            # the segment endpoints that straddle the box: one in, one out
            faces = set()
            for s in segs:
                p, q = (s.start_x, s.start_y), (s.end_x, s.end_y)
                if inside(p, b) != inside(q, b):
                    faces.add(face(q if inside(p, b) else p, b) + '/' + s.layer[0])
            out.append(f'{ref} {",".join(sorted(faces)) or "-"}')
        vd = []
        for v in vias:
            where = next((r for r, b in boxes.items() if inside((v.x, v.y), b)), 'open')
            vd.append(f'({v.x:.1f},{v.y:.1f}:{where})')
        out.append(f'vias {len(vias)} ' + ' '.join(vd))
        ln = {}
        for s in segs:
            ln[s.layer] = ln.get(s.layer, 0.0) + math.hypot(s.end_x - s.start_x,
                                                            s.end_y - s.start_y)
        out.append('  '.join(f'{L[0]} {d:.1f}mm' for L, d in sorted(ln.items())))
        print('  '.join(out))


if __name__ == '__main__':
    main()
