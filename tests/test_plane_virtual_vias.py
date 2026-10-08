#!/usr/bin/env python3
"""Every pad a plane net defers to the route step seeds its split -- a VIRTUAL VIA, never copper.

  python3 tests/test_plane_virtual_vias.py

A chain pours its planes before any fanout or routing (#562), so the pads are
the only sign of where each rail is used. Only the balls under a BGA used to
seed a split; every other deferred pad -- decoupling caps, a regulator's
output, an RF section's grounds -- seeded nothing whenever its net already had
copper on the layer (a through-hole pad), and the split was drawn round a few
points (zynq_ad9364: RFGND a hull round U5's balls, ~60 of its pads under GND).

On kicad_files/ulx3s.kicad_pcb, GND and +3V3 sharing In1.Cu (create_plane, the
GUI's path: dry run, results returned):

1. every deferred pad is recorded as a virtual via (the deferral's count), and
   the split takes every one as a point of its net's spines ("N virtual via(s)
   and M pad(s) on In1.Cu join its spines", summed over the nets);
2. each net's pads lie inside its own zone outlines -- +3V3, the island net, at
   least COVER of them (with only the under-BGA balls seeded, 28 of its 71 did);
3. a second call on the same pcb_data starts from no seeds: the GUI may pour
   twice on one board, and an earlier call's seeds must not stand in.

Uses kicad_files/ulx3s.kicad_pcb; skips cleanly if absent.
"""
import contextlib
import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
NETS = ['GND', '+3V3']
COVER = 0.85


def inside(poly, x, y):
    hit, n = False, len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def pour(pcb):
    import route_planes
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        res = route_planes.create_plane(
            input_file=BOARD, output_file='', net_names=NETS, plane_layers=['In1.Cu'] * len(NETS), pcb_data=pcb,
            dry_run=True, return_results=True, all_layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
            layer_nets={'In1.Cu': list(NETS)})
    return res[5], out.getvalue()


def main():
    print('=' * 60)
    print('virtual vias: every deferred pad seeds its split')
    print('=' * 60)
    if not os.path.exists(BOARD):
        print(f'  [SKIP] board not present: {BOARD}')
        return 0
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(BOARD)
    fails = []
    zones, log = pour(pcb)
    deferred = sum(int(n) for n in re.findall(r'(\d+) pad\(s\) on \'[^\']+\' deferred', log))
    deferred += len(re.findall(r'thermal array did not fit', log))
    seeds = {nid: list(v) for nid, v in (getattr(pcb, '_deferred_pad_seeds', None) or {}).items()}
    nseeds = sum(len(v) for v in seeds.values())
    taken = sum(int(n) for n in re.findall(r'(\d+) virtual via\(s\) and \d+ pad\(s\) on \S+ join its spines', log))
    print(f'  deferred pads {deferred}, virtual vias recorded {nseeds}, taken by the split {taken}')
    if deferred == 0:
        fails.append('no pad was deferred -- the arms below test nothing')
    if nseeds != deferred:
        fails.append(f'{deferred} pads deferred but {nseeds} virtual vias recorded: each deferred pad is one')
    if taken != len({(nid, p) for nid, v in seeds.items() for p in v}):
        fails.append(f'{nseeds} virtual vias recorded but the split took {taken}: each joins its net\'s spines')
    for nm in NETS:
        nid = next(i for i, n in pcb.nets.items() if n.name == nm)
        polys = [z['polygon_points'] for z in zones if z.get('net_id') == nid]
        pads = pcb.pads_by_net.get(nid, [])
        ins = sum(1 for p in pads if any(inside(pp, p.global_x, p.global_y) for pp in polys))
        print(f'  {nm}: {len(polys)} zone polygon(s), {ins} of its {len(pads)} pads inside its own outline')
        if pads and ins < COVER * len(pads):
            fails.append(f'{nm}: {ins} of {len(pads)} pads inside its own outline, want at least {COVER:.0%}')
    _zones2, _log2 = pour(pcb)
    nseeds2 = sum(len(v) for v in (getattr(pcb, '_deferred_pad_seeds', None) or {}).values())
    if nseeds2 != nseeds:
        fails.append(f'a second pour on the same pcb_data recorded {nseeds2} virtual vias, the first {nseeds}: '
                     f'an earlier call\'s seeds stood in')
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: every deferred pad seeds its split, each net covers its pads, no seed carried between calls')
    return 0


if __name__ == '__main__':
    sys.exit(main())
