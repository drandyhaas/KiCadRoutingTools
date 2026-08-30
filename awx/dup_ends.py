#!/usr/bin/env python3
"""Stub free ends that two nets share (within 0.05 mm) -- the fanout
artefact that put K21's SRAS (B) and SCKE1 (F) ends at one point.
usage: dup_ends.py BOARD [REF]   (REF: only that footprint's nets' ends)"""
import math
import os
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

pcb = parse_kicad_pcb(sys.argv[1])
ref = sys.argv[2] if len(sys.argv) > 2 else None
name = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
ends = []
for nid, net in pcb.nets.items():
    if ref and not any(p.component_ref == ref for p in net.pads):
        continue
    segs = [s for s in pcb.segments if s.net_id == nid]
    if not segs:
        continue
    cnt = Counter()
    for s in segs:
        cnt[(round(s.start_x, 3), round(s.start_y, 3))] += 1
        cnt[(round(s.end_x, 3), round(s.end_y, 3))] += 1
    anchors = [(p.global_x, p.global_y, max(p.size_x, p.size_y) / 2) for p in net.pads] + \
        [(v.x, v.y, v.size / 2) for v in pcb.vias if v.net_id == nid]
    for pt, c in cnt.items():
        if c == 1 and all(math.hypot(pt[0] - ax, pt[1] - ay) > max(0.02, ar)
                          for (ax, ay, ar) in anchors):
            lay = next(s.layer for s in segs
                       if (round(s.start_x, 3), round(s.start_y, 3)) == pt
                       or (round(s.end_x, 3), round(s.end_y, 3)) == pt)
            ends.append((pt, nid, lay))
dups = []
for i in range(len(ends)):
    for j in range(i + 1, len(ends)):
        (a, na, la), (b, nb, lb) = ends[i], ends[j]
        if na != nb and math.hypot(a[0] - b[0], a[1] - b[1]) <= 0.05:
            dups.append(f'{name[na]}({la[0]}) & {name[nb]}({lb[0]}) at ({a[0]:.3f},{a[1]:.3f})')
print(f'{len(ends)} free ends, {len(dups)} shared point(s)'
      + (': ' + '; '.join(dups) if dups else ''))
