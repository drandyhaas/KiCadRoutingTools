#!/usr/bin/env python3
"""Compare raw vs octified same-layer gaps for a net pair in an x window."""
import json
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
import topo_strings as ts  # noqa: E402

na, nb, xlo, xhi = sys.argv[1], sys.argv[2], float(sys.argv[3]), \
    float(sys.argv[4])
for tag, fn in (('raw', 'k8_raw.json'), ('octi', 'k8_octi.json')):
    d = json.load(open(fn))
    best = (9e9, None, None)
    for (p, q, la) in d[na]:
        if max(p[0], q[0]) < xlo or min(p[0], q[0]) > xhi:
            continue
        for (c, e, lb) in d[nb]:
            if la != lb:
                continue
            if max(c[0], e[0]) < xlo or min(c[0], e[0]) > xhi:
                continue
            dist = ts.seg_seg_dist(tuple(p), tuple(q), tuple(c), tuple(e))
            if dist < best[0]:
                best = (dist, (p, q, la), (c, e, lb))
    dist, s1, s2 = best
    print(f'{tag}: min gap {dist:.3f}')
    print(f'  {na}: {s1}')
    print(f'  {nb}: {s2}')
