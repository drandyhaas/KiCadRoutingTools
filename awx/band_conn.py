#!/usr/bin/env python3
"""Is a lane's BAND connected from its tooth to its stub, with no copper
in the way at all? A flood over the band mask (8-connected within a
layer, a layer change wherever both layers are allowed) from the tooth
cell; reports whether the stub cell is reached and, when not, how far
along the spine the flood got and the last cells it held.
usage: band_conn.py FANOUT_BOARD K NET [--dest REF]"""
import argparse
import os
import subprocess
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import braid as te  # noqa: E402
from band_check import window_axes  # noqa: E402
from schedule import Schedule  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('net')
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--copper', action='store_true',
                    help="flood band AND NOT the router's static obstacle map")
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    quiet = lambda *x: None  # noqa: E731
    ctx, groups = te.setup(a.board, names, a.dest, quiet)
    for gi, grp in enumerate(groups):
        if a.net not in grp:
            continue
        c = te.Corridor(gi, grp, ctx, quiet)
        c.build_spine()
        c.classify()
        c.offsets(0.35)
        c.reserve_intervals()
        sched = Schedule(c.launch, c.target, ctx.tooth_layer,
                     dest_layer=ctx.dest_layer)
        cols, gate = c.plan_columns(sched, {d: 1 for d in sched.divers},
                                    {d: 0 for d in sched.divers})
        c.lay_lanes(cols)
        nm = a.net
        xs, ys = window_axes(c, nm)
        band = c.band_of(nm)
        masks = {L: band(xs, ys, L) for L in ('F.Cu', 'B.Cu')}
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        S, O = c.spine.project(X, Y)
        li = {'F.Cu': 0, 'B.Cu': 1}
        M = np.stack([masks['F.Cu'], masks['B.Cu']])
        if a.copper:
            # the router's own obstacle map over connect()'s window
            # (static copper only, no virtual), read back cell by cell
            from routing_config import GridCoord
            from plane_pad_tap import make_local_window
            from obstacle_map import build_base_obstacle_map
            from net_rescue import _fence_window
            cfg, pcb = ctx.cfg, ctx.pcb
            nid = ctx.byname[nm][0]
            coord = GridCoord(cfg.grid_step)
            pts = [c.teeth[nm], c.stubs[nm]] + list(c.lane_xy[nm])
            bx0, bx1 = min(p[0] for p in pts), max(p[0] for p in pts)
            by0, by1 = min(p[1] for p in pts), max(p[1] for p in pts)
            cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
            half = max(bx1 - bx0, by1 - by0) / 2 + 0.6
            window = make_local_window(pcb, cx, cy, half)
            obstacles = build_base_obstacle_map(window, cfg, [nid])
            _fence_window(obstacles, window, cfg)
            blocked = np.zeros(M.shape, dtype=bool)
            for L in (0, 1):
                for i, x in enumerate(xs):
                    gx = coord.to_grid(float(x), 0.0)[0]
                    for j, y in enumerate(ys):
                        gy = coord.to_grid(0.0, float(y))[1]
                        if obstacles.is_blocked(gx, gy, L):
                            blocked[L][i, j] = True
            print(f'  copper: {int(blocked.sum())} blocked cells in the window; '
                  f'band cells lost to copper: F {int((M[0] & blocked[0]).sum())}, '
                  f'B {int((M[1] & blocked[1]).sum())}')
            M = M & ~blocked

        def cell(p):
            return (int(round((p[0] - xs[0]) / 0.025)), int(round((p[1] - ys[0]) / 0.025)))
        t = cell(c.teeth[nm])
        e = cell(c.stubs[nm])
        tl, el = li[ctx.tooth_layer[nm]], li[ctx.dest_layer[nm]]
        print(f'corridor {gi}: gate {gate}, {len(cols)} columns; {nm} tooth cell {t} '
              f'{ctx.tooth_layer[nm]} allowed={bool(M[tl][t])}, stub cell {e} '
              f'{ctx.dest_layer[nm]} allowed={bool(M[el][e])}')
        seen = np.zeros(M.shape, dtype=bool)
        q = deque()
        if M[tl][t]:
            seen[tl][t] = True
            q.append((tl, t[0], t[1]))
        nx, ny = M.shape[1], M.shape[2]
        while q:
            L, i, j = q.popleft()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    ii, jj = i + di, j + dj
                    if 0 <= ii < nx and 0 <= jj < ny and M[L][ii, jj] and not seen[L][ii, jj]:
                        seen[L][ii, jj] = True
                        q.append((L, ii, jj))
            oL = 1 - L
            if M[oL][i, j] and not seen[oL][i, j]:
                seen[oL][i, j] = True
                q.append((oL, i, j))
        reached = seen[el][e]
        print(f'  stub reached: {reached}')
        for L in (0, 1):
            ii, jj = np.nonzero(seen[L])
            if len(ii) == 0:
                print(f'  layer {L}: nothing reached')
                continue
            ss = S[ii, jj]
            k = np.argmax(ss)
            print(f'  layer {L}: {len(ii)} cells reached, s up to {ss[k]:.3f} at '
                  f'(s,o)=({ss[k]:.3f},{O[ii[k], jj[k]]:.3f}) xy=({xs[ii[k]]:.3f},{ys[jj[k]]:.3f})')
        # where does the band continue past the flood? the allowed cells
        # of the lane just beyond the frontier
        front = max(float(S[np.nonzero(seen[L])].max()) for L in (0, 1)
                    if seen[L].any())
        for L in (0, 1):
            ii, jj = np.nonzero(M[L] & ~seen[L] & (S > front) & (S < front + 0.3))
            if len(ii):
                ss, oo = S[ii, jj], O[ii, jj]
                k = np.argmin(ss)
                print(f'  layer {L}: next allowed cells beyond the frontier from '
                      f'(s,o)=({ss[k]:.3f},{oo[k]:.3f}); o range there '
                      f'[{oo[ss < ss[k] + 0.03].min():.3f},{oo[ss < ss[k] + 0.03].max():.3f}]')
        ms = [p[0] for p in c.mid[nm]]
        mo = [p[1] for p in c.mid[nm]]
        print(f'  lane o at frontier s={front:.3f}: {np.interp(front, ms, mo):.3f}; '
              f'mid pieces around it: '
              + str([(round(s, 2), round(float(o), 3)) for s, o in zip(ms, mo)
                     if front - 0.5 < s < front + 0.5]))
        return


if __name__ == '__main__':
    main()
