#!/usr/bin/env python3
"""What is already fanned out on the bench, and where do the stubs end?

"Feed the plan into the production fanout" means different work
depending on the answer: a component with stubs already on it has to
have them ripped and re-laid, while a bare one just gets fanned out.
And the plan's two ends are not symmetric -- the source end is planned
from an EXISTING tooth, the destination end from the raw ball -- so
this checks which is which rather than assuming.

usage: probe_fanout_state.py [K]
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '51'
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)

print(f'board: {len(pcb.segments)} segments, {len(pcb.vias)} vias, '
      f'{len(pcb.footprints)} footprints')

# which footprint does each net's copper sit near?
near = {}
for nm in names:
    nid = byname[nm][0]
    segs = [s for s in pcb.segments if s.net_id == nid]
    vias = [v for v in pcb.vias if v.net_id == nid]
    pads = byname[nm][1].pads
    per = {}
    for p in pads:
        r = p.component_ref
        c = sum(1 for s in segs
                if min((s.start_x - p.global_x) ** 2
                       + (s.start_y - p.global_y) ** 2,
                       (s.end_x - p.global_x) ** 2
                       + (s.end_y - p.global_y) ** 2) < 4.0)
        per[r] = per.get(r, 0) + c
    near[nm] = (len(segs), len(vias), per)

refs = {}
for nm in names:
    for r, c in near[nm][2].items():
        a, b = refs.get(r, (0, 0))
        refs[r] = (a + c, b + (1 if c else 0))
print('\nsegments within 2 mm of a pad, by component:')
for r, (c, n) in sorted(refs.items(), key=lambda kv: -kv[1][0]):
    print(f'  {r:6s}: {c:4d} segments near, on {n} of {len(names)} nets')

tot_s = sum(v[0] for v in near.values())
tot_v = sum(v[1] for v in near.values())
print(f'\nthe {len(names)} planned nets carry {tot_s} segments, '
      f'{tot_v} vias between them '
      f'({tot_s / len(names):.1f} segments per net)')
sample = names[:6]
print('\nper net (first 6): stub end, and the pad the plan calls the '
      'destination')
for nm in sample:
    s, v, per = near[nm]
    src, dst, dref = ends[nm]
    print(f'  {nm:8s} {s:2d} seg {v} via   stub end '
          f'({src[0]:7.2f},{src[1]:7.2f})   dest {dref} '
          f'({dst[0]:7.2f},{dst[1]:7.2f})   near={per}')
