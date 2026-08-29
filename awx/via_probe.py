#!/usr/bin/env python3
"""Which segment of a net is closest to a via point?"""
import json
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
import topo_strings as ts  # noqa: E402

fn, nm, vx, vy = sys.argv[1], sys.argv[2], float(sys.argv[3]), \
    float(sys.argv[4])
d = json.load(open(fn))
rows = []
for (p, q, la) in d[nm]:
    dist = ts.seg_pt_dist(tuple(p), tuple(q), (vx, vy))
    rows.append((dist, p, q, la))
rows.sort()
for dist, p, q, la in rows[:4]:
    print(f'{dist:.3f} ({p[0]:.3f},{p[1]:.3f})->({q[0]:.3f},{q[1]:.3f}) '
          f'{la}')
