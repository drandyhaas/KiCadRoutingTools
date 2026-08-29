#!/usr/bin/env python3
"""Print the first K nets of the COHERENT K-ladder (k_ladder_coherent.txt).

That file's own header is the point: a prefix K must never split a
river. Taking "the first K nets by launch y" instead mixes singletons
into small K and measures a harder problem than the campaign's -- at
K=8 the braid emits 593 DRC violations on the launch-y prefix and 0 on
this one."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
rivers = []
for line in open(os.path.join(HERE, 'k_ladder_coherent.txt')):
    line = line.strip()
    if not line or line.startswith('#'):
        continue
    rivers.append(line.split())
flat = [n for r in rivers for n in r]
if '--checkpoints' in sys.argv:
    tot = 0
    out = []
    for r in rivers:
        tot += len(r)
        out.append(str(tot))
    print(' '.join(out))
else:
    K = int(sys.argv[1]) if len(sys.argv) > 1 else 51
    print(','.join(flat[:K]))
