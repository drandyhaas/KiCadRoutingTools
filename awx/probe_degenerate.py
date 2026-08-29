#!/usr/bin/env python3
"""Count degenerate copper on a board: zero-length segments and exact
duplicate segments, per net. They are electrically harmless but they
are junk in the output, they inflate the segment count, and they make
any structural audit read a clean chain as a branched one."""
import math
import os
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

pcb = parse_kicad_pcb(sys.argv[1])
names = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
zero = Counter()
dup = Counter()
seen = set()
for s in pcb.segments:
    if math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) < 1e-6:
        zero[names.get(s.net_id, s.net_id)] += 1
        continue
    k = (round(s.start_x, 4), round(s.start_y, 4),
         round(s.end_x, 4), round(s.end_y, 4), s.layer, s.net_id)
    kr = (k[2], k[3], k[0], k[1], k[4], k[5])
    if k in seen or kr in seen:
        dup[names.get(s.net_id, s.net_id)] += 1
    seen.add(k)
print(f'{os.path.basename(sys.argv[1])}: {len(pcb.segments)} segments')
print(f'  zero-length: {sum(zero.values())}'
      + (f'  {dict(zero)}' if zero else ''))
print(f'  duplicates : {sum(dup.values())}'
      + (f'  {dict(dup)}' if dup else ''))
