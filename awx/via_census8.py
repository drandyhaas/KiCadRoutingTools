#!/usr/bin/env python3
"""Per-net via/layer census for the K8 true-bus goal: our board vs the
human original, same 8 nets."""
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

K8 = ['SDQ15', 'SDQ14', 'SDQ13', 'SDQ11',
      'SDQ0', 'SDQM0', 'SDQ12', 'SDQ8']


def census(path, label, base=None):
    pcb = parse_kicad_pcb(path)
    basev = set()
    bases = set()
    if base is not None:
        bpcb = parse_kicad_pcb(base)
        basev = {(round(v.x, 3), round(v.y, 3)) for v in bpcb.vias}
        bases = {(round(s.start_x, 3), round(s.start_y, 3),
                  round(s.end_x, 3), round(s.end_y, 3))
                 for s in bpcb.segments}
    print(f'--- {label}')
    tot_v = 0
    for nm in K8:
        nid = next((i for i, n in pcb.nets.items()
                    if n.name.endswith('/' + nm)), None)
        if nid is None:
            print(f'  {nm}: NET NOT FOUND')
            continue
        vias = [v for v in pcb.vias if v.net_id == nid
                and (round(v.x, 3), round(v.y, 3)) not in basev]
        segs = [s for s in pcb.segments if s.net_id == nid
                and (round(s.start_x, 3), round(s.start_y, 3),
                     round(s.end_x, 3), round(s.end_y, 3)) not in bases]
        layers = {}
        for s in segs:
            L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
            layers[s.layer] = layers.get(s.layer, 0.0) + L
        lay = ' '.join(f'{k}:{v:.1f}mm' for k, v in sorted(layers.items()))
        tot_v += len(vias)
        vpos = ' '.join(f'({v.x:.1f},{v.y:.1f})' for v in vias)
        print(f'  {nm}: {len(vias)} vias {vpos}  [{lay}]')
    print(f'  TOTAL new vias on the 8: {tot_v}')


if __name__ == '__main__':
    census(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else sys.argv[1],
           base=sys.argv[3] if len(sys.argv) > 3 else None)
