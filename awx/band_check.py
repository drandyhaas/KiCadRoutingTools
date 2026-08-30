#!/usr/bin/env python3
"""Exactness check for Corridor.band_of: the 1-D-in-s band against a
per-cell reference (the original formulation, kept here verbatim) on
every lane's own window and both layers, plus the time of each.
usage: band_check.py FANOUT_BOARD K [--dest REF]"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import braid as te  # noqa: E402
from schedule import Schedule  # noqa: E402


def ref_band_of(c, nm):
    """The per-cell original."""
    sp = c.spine
    HALF_SEP = te.HALF_SEP

    def band(xs, ys, L):
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        S, O = sp.project(X, Y)
        ms = np.array([p[0] for p in c.mid[nm]])
        mo = np.array([p[1] for p in c.mid[nm]])
        present = (S >= ms[0] - 1e-9) & (S <= ms[-1] + 1e-9)
        o_nm = np.interp(S, ms, mo)
        okL = c.allowed_vec(nm, S, L)
        prev = np.full(S.shape, -np.inf)
        nxt = np.full(S.shape, np.inf)
        for om in c.members:
            if om == nm:
                continue
            os_ = np.array([p[0] for p in c.mid[om]])
            oo_ = np.array([p[1] for p in c.mid[om]])
            pres = (S >= os_[0] - 1e-9) & (S <= os_[-1] + 1e-9)
            if not pres.any():
                continue
            o_m = np.interp(S, os_, oo_)
            m = pres & c.allowed_vec(om, S, L)
            prev = np.where(m & (o_m < o_nm), np.maximum(prev, o_m), prev)
            nxt = np.where(m & (o_m > o_nm), np.minimum(nxt, o_m), nxt)
        lo = np.where(np.isfinite(prev), (prev + o_nm) / 2 + HALF_SEP, -np.inf)
        hi = np.where(np.isfinite(nxt), (nxt + o_nm) / 2 - HALF_SEP, np.inf)
        lo = np.minimum(lo, o_nm - 0.03)
        hi = np.maximum(hi, o_nm + 0.03)
        ok = present & okL & (O >= lo) & (O <= hi)
        for (s_l, oa, ob) in c.legs[nm]:
            rect = ((np.abs(S - s_l) <= te.LEG_W)
                    & (O >= min(oa, ob) - te.LEG_O) & (O <= max(oa, ob) + te.LEG_O))
            ok |= rect & okL
        for ((sa, oa), (sb, ob)) in c.jogs.get(nm, ()):
            rect = ((S >= min(sa, sb) - te.LEG_O) & (S <= max(sa, sb) + te.LEG_O)
                    & (np.abs(O - oa) <= te.LEG_O))
            ok |= rect & okL
        for (a, b) in c.cross_iv:
            ok |= ((S >= a) & (S <= b) & present
                   & (np.abs(O - o_nm) <= te.CROSS_TUBE))
        return ok
    return band


def window_axes(c, nm, margin=0.6, step=0.025):
    pts = [c.teeth[nm], c.stubs[nm]] + list(c.lane_xy[nm])
    bx0, bx1 = min(p[0] for p in pts), max(p[0] for p in pts)
    by0, by1 = min(p[1] for p in pts), max(p[1] for p in pts)
    cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
    half = max(bx1 - bx0, by1 - by0) / 2 + margin
    gx0, gx1 = int(np.floor((cx - half) / step)), int(np.ceil((cx + half) / step))
    gy0, gy1 = int(np.floor((cy - half) / step)), int(np.ceil((cy + half) / step))
    xs = np.arange(gx0, gx1 + 1) * step
    ys = np.arange(gy0, gy1 + 1) * step
    return xs, ys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--dest', default='DU1')
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    quiet = lambda *x: None  # noqa: E731
    ctx, groups = te.setup(a.board, names, a.dest, quiet)
    t_new = t_ref = 0.0
    worst = (0, None)
    total_cells = 0
    n_diff_lanes = 0
    for gi, grp in enumerate(groups):
        c = te.Corridor(gi, grp, ctx, quiet)
        c.build_spine()
        c.classify()
        if c.s1 - c.s0 < 0.5:
            continue
        c.offsets(0.35)
        c.reserve_intervals()
        sched = Schedule(c.launch, c.target, ctx.tooth_layer,
                     dest_layer=ctx.dest_layer)
        cols = sched.columns({d: 1 for d in sched.divers}, {d: 0 for d in sched.divers})
        c.lay_lanes(cols)
        for nm in grp:
            xs, ys = window_axes(c, nm)
            new = c.band_of(nm)
            ref = ref_band_of(c, nm)
            diff_lane = 0
            for L in ('F.Cu', 'B.Cu'):
                t0 = time.perf_counter()
                m_new = new(xs, ys, L)
                t_new += time.perf_counter() - t0
                t0 = time.perf_counter()
                m_ref = ref(xs, ys, L)
                t_ref += time.perf_counter() - t0
                d = int(np.count_nonzero(m_new != m_ref))
                diff_lane += d
                total_cells += m_new.size
                if d > worst[0]:
                    worst = (d, (gi, nm, L, m_new.size))
            if diff_lane:
                n_diff_lanes += 1
                print(f'  corridor {gi} {nm}: {diff_lane} cell(s) differ')
    print(f'{total_cells} cells over every lane and layer: '
          f'{n_diff_lanes} lane(s) differ, worst {worst}')
    print(f'time: new {t_new:.2f} s, reference {t_ref:.2f} s')


if __name__ == '__main__':
    main()
