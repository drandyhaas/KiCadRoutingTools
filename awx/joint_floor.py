#!/usr/bin/env python3
"""#622 THE JOINT FLOOR -- the non-circular version of the layer floor.

`ledger_cal` prices each net with every OTHER net pinned at the layer it
ACTUALLY has on the board, so on a badly realized board the partners
alternate and each net's "floor" alternates with them: the instrument
moves with the thing it is meant to be independent of. Measured, that
per-net floor is 20-40 vias LOOSE on exactly the boards a search walks
past and rejects (ours K41 at 130 vias: per-net floor 108, joint 88).

The joint floor asks the honest question instead. Over the SAME fixed
paths -- nobody reroutes anything -- choose the layer of every path at
every crossing at once, subject to:

  * two paths that cross must be on OPPOSITE layers there (that is what
    a crossing MEANS: same layer would be a short),
  * each path starts and ends on its own PAD's layer,

and minimise the total number of layer changes, which is the vias. That
is a lower bound no realization of these paths can beat, and unlike the
per-net floor nothing in it is conditioned on the answer.

MODEL. Per crossing j on path m a binary y[m,j] (1 = B.Cu) -- the layer
AT that crossing, which is what both the constraint and `ledger_cal._dp`
talk about. A crossing between m's j-th and o's k-th is the equality
y[m,j] + y[o,k] = 1. Changes along a path are the adjacencies of its own
sorted crossings, with the two pad layers pinned at the ends; each is a
`d >= |difference|` pair, continuous, driven to the difference because
the objective is positive. Exact, and small: K51's 352 crossings give
~700 binaries and ~350 equalities.

INFEASIBLE IS A RESULT, not an error. The equalities are a parity system
and an odd cycle in it has no assignment -- which says these paths cannot
be two-layer realized AT ALL, whatever the vias. It is reported as such.

    python3 joint_floor.py BOARD K [--src U1] [--dst DU1]
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

import numpy as np
from scipy.optimize import milp, Bounds, LinearConstraint
from scipy.sparse import coo_matrix

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from ledger_cal import Judge, _cross_pair          # noqa: E402

# the same node budget discipline as the braid: a bound, never a clock.
NODES = int(os.environ.get('JOINT_FLOOR_NODES', '200000'))


def joint_floor(J):
    """(floor, per-net changes, status) for a ledger_cal.Judge's paths.
    `floor` is None when the parity system is infeasible."""
    names = sorted(J.paths)
    # every crossing, as a pair of (net, arclength) sites
    sites = {m: [] for m in names}          # net -> [t] in path order
    pairs = []                              # (m, ta, o, tb)
    for i, m in enumerate(names):
        for o in names[i + 1:]:
            for (ta, tb) in _cross_pair(J.paths[m], J.paths[o]):
                pairs.append((m, ta, o, tb))
                sites[m].append(ta)
                sites[o].append(tb)
    for m in names:
        sites[m].sort()
    # y[(m, t)] -- one binary per crossing SITE
    idx = {}
    for m in names:
        for t in sites[m]:
            idx.setdefault((m, t), len(idx))
    ny = len(idx)
    rows, lb, ub = [], [], []
    cost = np.zeros(ny)                     # the d variables are appended

    def add(co, lo, hi):
        rows.append(co); lb.append(lo); ub.append(hi)

    # 1. a crossing puts the two paths on opposite layers
    for (m, ta, o, tb) in pairs:
        add({idx[(m, ta)]: 1, idx[(o, tb)]: 1}, 1, 1)
    # 2. the changes along each path, pads pinned at both ends
    nv = ny
    dcost = []
    for m in names:
        P = J.paths[m]
        chain = sites[m]
        prev_pin = 1 if P.start_pad_lay == 'B.Cu' else 0
        for pos, t in enumerate(chain):
            v = idx[(m, t)]
            d = nv; nv += 1; dcost.append(1.0)
            if pos == 0:
                # |y - start_pad|, a constant on one side
                add({d: 1, v: -1}, -prev_pin, np.inf)
                add({d: 1, v: 1}, prev_pin, np.inf)
            else:
                u = idx[(m, chain[pos - 1])]
                add({d: 1, v: -1, u: 1}, 0, np.inf)
                add({d: 1, v: 1, u: -1}, 0, np.inf)
        end_pin = 1 if P.end_pad_lay == 'B.Cu' else 0
        d = nv; nv += 1; dcost.append(1.0)
        if chain:
            v = idx[(m, chain[-1])]
            add({d: 1, v: -1}, -end_pin, np.inf)
            add({d: 1, v: 1}, end_pin, np.inf)
        else:
            # a path nothing crosses: it changes only if its two pads
            # disagree, which is a constant, not a decision
            add({d: 1}, abs(end_pin - prev_pin), np.inf)
    cvec = np.concatenate([cost, np.asarray(dcost, float)])
    integ = np.concatenate([np.ones(ny), np.zeros(len(dcost))])
    ri, ci, vi = [], [], []
    for i, co in enumerate(rows):
        for k, v in co.items():
            ri.append(i); ci.append(k); vi.append(float(v))
    A = coo_matrix((vi, (ri, ci)), shape=(max(1, len(rows)), nv))
    res = milp(cvec, constraints=LinearConstraint(A.tocsr(),
                                                  np.asarray(lb, float),
                                                  np.asarray(ub, float)),
               integrality=integ, bounds=Bounds(0, 1),
               options={'node_limit': NODES})
    if res.x is None:
        return None, {}, (res.message or 'no solution')
    x = res.x
    per = {}
    off = ny
    for m in names:
        n_ = len(sites[m]) + 1
        per[m] = int(round(sum(x[off:off + n_])))
        off += n_
    return int(round(float(cvec @ x))), per, 'optimal'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k')
    ap.add_argument('--src', default='U1')
    ap.add_argument('--dst', default='DU1')
    ap.add_argument('--verbose', action='store_true')
    a = ap.parse_args()
    nets = subprocess.run(
        # NOTE: no --board. The net list must be the one ledger_cal uses
        # (the BENCH's), or the two floors are computed over different
        # sets and the comparison that motivates this file is void --
        # measured: --board=<the routed output> gave 10 paths of 35.
        [sys.executable, os.path.join(HERE, 'coherent_nets.py'), a.k],
        capture_output=True, text=True).stdout.strip().split(',')
    nets = [n for n in nets if n]
    J = Judge(a.board, nets, a.src, a.dst)
    missing = [m for m in nets if m not in J.paths]
    for m in missing:
        print(f'  (no path for {m})')
    floor, per, status = joint_floor(J)
    n_cross = sum(len(v) for v in J.seqs.values()) // 2
    print(f'{os.path.basename(a.board)} K={a.k}: {len(J.paths)} path(s), '
          f'{n_cross} crossing(s)')
    print(f'  routed changes   {J.act_total}')
    print(f'  per-net floor    {J.floor_total}   (ledger_cal: CIRCULAR -- '
          f'each net priced against the others AS LAID)')
    if floor is None:
        print(f'  JOINT floor      INFEASIBLE ({status}) -- these paths have '
              f'no two-layer assignment at all')
        return 2
    print(f'  JOINT floor      {floor}   slack {J.act_total - floor}')
    if a.verbose:
        # the nets with the most headroom first: that is where a
        # realization idea has room to pay
        for m in sorted(per, key=lambda k_: (per[k_] - J.paths[k_].changes, k_)):
            print(f'    {m:10s} routed {J.paths[m].changes:3d}  '
                  f'per-net {J.per[m]:3d}  joint {per[m]:3d}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
