#!/usr/bin/env python3
"""copper_same.py A.kicad_pcb B.kicad_pcb -- are the two boards' copper
IDENTICAL as sets (segments by start/end/layer/width/net name, vias by
xy/size/drill/layers/net name)? UUIDs differ run to run, so a file diff
cannot say; this can. Exit 0 identical, 1 different (the differences
counted)."""
import sys
import os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, os.path.join(HERE, '..'))
from kicad_parser import parse_kicad_pcb
def key(p):
    pcb = parse_kicad_pcb(p)
    nm = {i: n.name for i, n in pcb.nets.items()}
    segs = sorted((round(s.start_x, 6), round(s.start_y, 6), round(s.end_x, 6), round(s.end_y, 6), s.layer, round(s.width, 6), nm.get(s.net_id, s.net_id)) for s in pcb.segments)
    vias = sorted((round(v.x, 6), round(v.y, 6), round(v.size, 6), round(v.drill, 6), tuple(v.layers), nm.get(v.net_id, v.net_id)) for v in pcb.vias)
    return segs, vias
sa, va = key(sys.argv[1]); sb, vb = key(sys.argv[2])
ds = len(set(sa) ^ set(sb)); dv = len(set(va) ^ set(vb))
print(f'segments {len(sa)} vs {len(sb)} ({ds} differ), vias {len(va)} vs {len(vb)} ({dv} differ): '
      + ('IDENTICAL copper' if ds == 0 and dv == 0 and len(sa) == len(sb) and len(va) == len(vb) else 'DIFFERENT'))
# the length test is in the verdict string AND in the exit code: a board
# that duplicates a segment has the same SET and a different board, and
# a caller following the docstring ("exit 0 identical") read DIFFERENT as
# identical
sys.exit(0 if ds == 0 and dv == 0 and len(sa) == len(sb) and len(va) == len(vb) else 1)
