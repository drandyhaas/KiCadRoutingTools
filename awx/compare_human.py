#!/usr/bin/env python3
"""Our routed board against the human original on the SAME nets (a
coherent-ladder checkpoint): vias, copper length per layer, and the
per-net via counts side by side, worst offenders first.
usage: compare_human.py OUR_BOARD K [--human BOARD] [--top N]"""
import argparse
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402


def census(pcb, names):
    byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}
    out = {}
    for nm in names:
        nid = byname.get(nm)
        if nid is None:
            out[nm] = None
            continue
        v = sum(1 for x in pcb.vias if x.net_id == nid)
        ln = {}
        for s in pcb.segments:
            if s.net_id == nid:
                ln[s.layer] = ln.get(s.layer, 0.0) + math.hypot(
                    s.end_x - s.start_x, s.end_y - s.start_y)
        out[nm] = (v, ln)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--human', default=os.path.expanduser(
        '~/Downloads/bus/00_human_original.kicad_pcb'))
    ap.add_argument('--top', type=int, default=8)
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    ours = census(parse_kicad_pcb(a.board), names)
    hum = census(parse_kicad_pcb(a.human), names)

    def tot(c):
        v = sum(x[0] for x in c.values() if x)
        L = {}
        for x in c.values():
            if x:
                for k, d in x[1].items():
                    L[k] = L.get(k, 0.0) + d
        return v, L
    ov, oL = tot(ours)
    hv, hL = tot(hum)
    layers = sorted(set(oL) | set(hL))
    print(f'K={a.k} ({len(names)} nets)   {"":8} {"ours":>8} {"human":>8}')
    print(f'  vias                      {ov:8d} {hv:8d}   ({ov / len(names):.2f} / '
          f'{hv / len(names):.2f} per net)')
    for L in layers:
        print(f'  copper {L:<8} mm         {oL.get(L, 0):8.1f} {hL.get(L, 0):8.1f}')
    print(f'  copper total mm           {sum(oL.values()):8.1f} {sum(hL.values()):8.1f}')
    rows = []
    for nm in names:
        o, h = ours[nm], hum[nm]
        if o is None or h is None:
            continue
        rows.append((o[0] - h[0], nm, o[0], h[0], sum(o[1].values()), sum(h[1].values())))
    rows.sort(key=lambda r: (-r[0], r[1]))
    print(f'  per net (vias ours/human, mm ours/human), we spend most first:')
    for d, nm, ov_, hv_, ol, hl in rows[:a.top]:
        print(f'    {nm:6} {ov_:2d}/{hv_:<2d}  {ol:5.1f}/{hl:<5.1f}')
    worse = sum(1 for r in rows if r[0] > 0)
    better = sum(1 for r in rows if r[0] < 0)
    print(f'  nets with more vias than the human: {worse}, fewer: {better}, '
          f'equal: {len(rows) - worse - better}')


if __name__ == '__main__':
    main()
