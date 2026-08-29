#!/usr/bin/env python3
"""Probe the K4 nets' endpoint geometry on the bench board:
pads per net (component, position, layer), stub segments near each pad,
and the free-end census (the tooth exits)."""
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

K4 = ['SDQ15', 'SDQ14', 'SDQ13', 'SDQ11']
pcb = parse_kicad_pcb(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'fb_t2q_base.kicad_pcb'))
print(f'copper layers: {pcb.board_info.copper_layers}')
print(f'bounds: {pcb.board_info.board_bounds}')
print(f'footprints: {len(pcb.footprints)}  segments: {len(pcb.segments)} '
      f'vias: {len(pcb.vias)}')
for ref, fp in sorted(pcb.footprints.items()):
    print(f'  {ref}: {fp.footprint_name} pads={len(fp.pads)} '
          f'at ({fp.x:.1f},{fp.y:.1f}) rot={fp.rotation}')

byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
for nm in K4:
    nid, net = byname[nm]
    print(f'\n{nm} (net {nid}): {len(net.pads)} pads')
    for p in net.pads:
        print(f'  pad {p.component_ref}.{p.pad_number} at '
              f'({p.global_x:.3f},{p.global_y:.3f}) layers={p.layers} '
              f'size=({p.size_x:.2f},{p.size_y:.2f}) drill={p.drill}')
    segs = [s for s in pcb.segments if s.net_id == nid]
    print(f'  segments: {len(segs)}')
    # endpoint census: free ends = points occurring once
    cnt = Counter()
    for s in segs:
        cnt[(round(s.start_x, 3), round(s.start_y, 3))] += 1
        cnt[(round(s.end_x, 3), round(s.end_y, 3))] += 1
    padpts = {(round(p.global_x, 3), round(p.global_y, 3)) for p in net.pads}
    free = [pt for pt, c in cnt.items() if c == 1 and pt not in padpts]
    for s in segs:
        print(f'    ({s.start_x:.3f},{s.start_y:.3f})->'
              f'({s.end_x:.3f},{s.end_y:.3f}) {s.layer} w={s.width}')
    print(f'  free ends (non-pad, count==1): {free}')
