#!/usr/bin/env python3
"""Histogram of very short segments per board, and the shortest few.
Sub-micron segments read as zero-length in any 3-decimal dump and make a
clean chain audit as a branched one."""
import math
import os
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

for b in sys.argv[1:]:
    pcb = parse_kicad_pcb(b)
    names = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
    hist = Counter()
    short = []
    for s in pcb.segments:
        L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        if L < 0.05:
            short.append((L, names.get(s.net_id), s.layer,
                          s.start_x, s.start_y))
        hist['<1um' if L < 0.001 else '<10um' if L < 0.01
             else '<50um' if L < 0.05 else 'ok'] += 1
    print(f'{os.path.basename(b)}: {len(pcb.segments)} segs  {dict(hist)}')
    for L, nm, lay, x, y in sorted(short)[:5]:
        print(f'    {L * 1000:9.4f} um  {nm} {lay} at ({x:.4f},{y:.4f})')
