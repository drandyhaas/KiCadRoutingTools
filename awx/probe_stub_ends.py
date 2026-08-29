#!/usr/bin/env python3
"""Census of U1-side stub free ends: how far past the BGA boundary does
each escape stub end, vs its ball's column depth?"""
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

pcb = parse_kicad_pcb(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'fb_t2q_base.kicad_pcb'))
u1 = pcb.footprints['U1']
u1_max_x = max(p.global_x for p in u1.pads)
print(f'U1 rightmost ball column x = {u1_max_x:.3f}')

rows = []
for nid, net in pcb.nets.items():
    u1_pads = [p for p in net.pads if p.component_ref == 'U1']
    if len(u1_pads) != 1 or len(net.pads) != 2:
        continue
    up = u1_pads[0]
    segs = [s for s in pcb.segments if s.net_id == nid]
    if not segs:
        continue
    cnt = Counter()
    for s in segs:
        cnt[(round(s.start_x, 3), round(s.start_y, 3))] += 1
        cnt[(round(s.end_x, 3), round(s.end_y, 3))] += 1
    padpts = {(round(p.global_x, 3), round(p.global_y, 3))
              for p in net.pads}
    free = [pt for pt, c in cnt.items() if c == 1 and pt not in padpts]
    if len(free) != 1:
        continue
    fx, fy = free[0]
    if fx < 124 or fx > 130:
        continue  # not an east-side escape
    depth_cols = round((u1_max_x - up.global_x) / 0.65)
    on_street = abs((fy - up.global_y)) > 0.1
    rows.append((fx - u1_max_x, depth_cols, on_street,
                 net.name.split('/')[-1], up.pad_number, fx, fy))

print(f'{"past(mm)":>9s} {"depth":>5s} {"street":>6s}  net      ball  end')
for past, d, st, nm, pn, fx, fy in sorted(rows):
    print(f'{past:9.3f} {d:5d} {str(st):>6s}  {nm:8s} {pn:5s} '
          f'({fx:.3f},{fy:.3f})')
print(f'\n{len(rows)} east-side stubs; grouped by overhang:')
for past, n in sorted(Counter(round(r[0], 2) for r in rows).items()):
    print(f'  +{past:.2f}mm past boundary: {n} stubs')
