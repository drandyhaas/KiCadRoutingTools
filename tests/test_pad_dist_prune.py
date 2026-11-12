"""_seg_foreign_pad_dist's pad prune returns the sampled minimum of the full sweep: random rounded-rect
pads (tilted, corner radii up to and past the half-extents, local-clearance and netclass excess) against a
brute-force evaluation of every sample point against every windowed foreign pad."""
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

import single_ended_routing as ser  # noqa: E402


def pads(rnd, n):
    nids = np.array([rnd.randint(1, 6) for _ in range(n)])
    cx = np.array([rnd.uniform(0, 10) for _ in range(n)])
    cy = np.array([rnd.uniform(0, 10) for _ in range(n)])
    hx = np.array([rnd.uniform(0.1, 1.0) for _ in range(n)])
    hy = np.array([rnd.uniform(0.1, 1.0) for _ in range(n)])
    cr = np.array([rnd.choice([0.0, 0.05, min(a, b), max(a, b)]) for a, b in zip(hx, hy)])
    rot = np.array([rnd.choice([0.0, 0.0, 45.0, 30.0, 90.0]) * math.pi / 180 for _ in range(n)])
    rc, rs = np.cos(rot), np.sin(rot)
    ex = np.abs(hx * rc) + np.abs(hy * rs)
    ey = np.abs(hx * rs) + np.abs(hy * rc)
    plc = np.array([rnd.choice([0.0, 0.0, 0.2]) for _ in range(n)])
    return nids, cx, cy, hx, hy, cr, rc, rs, ex, ey, plc, []


def brute(arr, net_id, x1, y1, x2, y2, base_clearance, net_clearances):
    nids, cx, cy, hx, hy, cr, rc, rs, ex, ey, plc, _ = arr
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02) + 1)
    R = ser._FOREIGN_PAD_WINDOW
    best = math.inf
    for k in range(len(cx)):
        if nids[k] == net_id:
            continue
        if (cx[k] + ex[k] < min(x1, x2) - R or cx[k] - ex[k] > max(x1, x2) + R or
                cy[k] + ey[k] < min(y1, y2) - R or cy[k] - ey[k] > max(y1, y2) + R):
            continue
        exc = 0.0
        if base_clearance is not None:
            exc = max(plc[k] - base_clearance, 0.0)
            if net_clearances:
                exc = max(exc, max(0.0, net_clearances.get(int(nids[k]), base_clearance) - base_clearance))
        for i in range(n + 1):
            t = np.linspace(0.0, 1.0, n + 1)[i]
            px, py = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
            ddx, ddy = px - cx[k], py - cy[k]
            lx = abs(ddx * rc[k] + ddy * rs[k])
            ly = abs(-ddx * rs[k] + ddy * rc[k])
            d = math.hypot(max(lx - (hx[k] - cr[k]), 0.0), max(ly - (hy[k] - cr[k]), 0.0)) - cr[k]
            best = min(best, d - exc)
    return best


def test_prune_matches_full_sweep(monkeypatch):
    rnd = random.Random(7)
    for trial in range(80):
        arr = pads(rnd, rnd.randint(1, 50))
        monkeypatch.setattr(ser, '_foreign_pad_arrays', lambda pcb, layer, _a=arr: _a)
        nc = {k: rnd.choice([0.1, 0.15, 0.3]) for k in range(1, 7)} if trial % 2 else None
        bc = None if trial % 3 == 0 else 0.1
        for _ in range(8):
            x1, y1 = rnd.uniform(1, 9), rnd.uniform(1, 9)
            x2, y2 = (x1, y1) if rnd.random() < 0.1 else (x1 + rnd.uniform(-3, 3), y1 + rnd.uniform(-3, 3))
            net = rnd.randint(1, 6)
            got = ser._seg_foreign_pad_dist(None, net, x1, y1, x2, y2, 'F.Cu', base_clearance=bc,
                                            net_clearances=nc)
            want = min(brute(arr, net, x1, y1, x2, y2, bc, nc), 1e9)   # 1e9: no custom pad in range
            assert abs(got - want) < 1e-9, (trial, got, want)


if __name__ == '__main__':
    class _MP:
        def setattr(self, o, n, v):
            setattr(o, n, v)
    test_prune_matches_full_sweep(_MP())
    print('PASS')
