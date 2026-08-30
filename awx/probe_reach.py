#!/usr/bin/env python3
"""Replay the corridor reach test (corridor.cluster_corridors) for ONE
net against the spine of the corridor it was split from: which of the
two runs fails (stub past the spine's end, tooth behind its start),
the run's endpoints, and the array pad it hits.
usage: probe_reach.py FANOUT_BOARD K NET [--dest REF]"""
import argparse
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import braid as te  # noqa: E402
import topo_strings as ts  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('net')
    ap.add_argument('--dest', default='DU1')
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    quiet = lambda *x: None  # noqa: E731
    ctx, groups = te.setup(a.board, names, a.dest, quiet)
    big = groups[0]
    if a.net in big:
        print(f'{a.net} is IN the main corridor already')
        return
    grp = list(big) + [a.net]
    sp = ctx.spine_of(grp, relax=False)
    ends = ctx.ends
    refs = set(ctx.src_ref.values()) | {ends[nm][2] for nm in names}
    pad_obs = te.array_pad_obstacles(ctx.pcb, refs)
    tooth, stub = ends[a.net][0], ends[a.net][1]
    P0, d0 = sp.P[0], sp.d[0]
    Pn, dn = sp.P[-1], sp.d[-1]
    t_e = (stub[0] - Pn[0]) * dn[0] + (stub[1] - Pn[1]) * dn[1]
    t_0 = (tooth[0] - P0[0]) * d0[0] + (tooth[1] - P0[1]) * d0[1]
    print(f'spine {sp.L:.2f} mm, start {tuple(round(v, 2) for v in P0)} dir '
          f'({d0[0]:.3f},{d0[1]:.3f}), end {tuple(round(v, 2) for v in Pn)} dir '
          f'({dn[0]:.3f},{dn[1]:.3f})')
    s_t, o_t = sp.project_pt(tooth)
    s_e, o_e = sp.project_pt(stub)
    print(f'{a.net}: tooth {tooth} (s,o)=({s_t:.2f},{o_t:.2f})  t_0={t_0:.2f}; '
          f'stub {stub} (s,o)=({s_e:.2f},{o_e:.2f})  t_e={t_e:.2f}')

    def report(tag, p, q):
        ok = pad_obs.seg_clear(p, q)
        print(f'  {tag}: run ({p[0]:.2f},{p[1]:.2f}) -> ({q[0]:.2f},{q[1]:.2f}) '
              f'{"clear" if ok else "HITS a pad"}')
        if not ok:
            best = None
            for ref in refs:
                for pad in ctx.pcb.footprints[ref].pads:
                    c = (pad.global_x, pad.global_y)
                    d = ts.seg_pt_dist(p, q, c)
                    if best is None or d < best[0]:
                        best = (d, ref, pad.pad_number, pad.net_name, c)
            d, ref, num, nn, c = best
            print(f'    nearest pad {ref}.{num} ({nn}) at ({c[0]:.2f},{c[1]:.2f}), '
                  f'{d:.3f} mm from the run')
    if t_e > 1.0:
        report('stub past the end', stub, (stub[0] - t_e * dn[0], stub[1] - t_e * dn[1]))
    else:
        print('  stub not past the end (no check)')
    if t_0 < -1.0:
        report('tooth behind the start', tooth,
               (tooth[0] - t_0 * d0[0], tooth[1] - t_0 * d0[1]))
    else:
        print('  tooth not behind the start (no check)')


if __name__ == '__main__':
    main()
