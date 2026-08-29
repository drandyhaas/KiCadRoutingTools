#!/usr/bin/env python3
"""Print the escape-move menu for each net's berth pad at a coherent
checkpoint, so the menu can be read before anything consumes it.

usage: probe_moves.py [K] [--net NAME]
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402
import escape_moves as em  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') \
    else '11'
only = None
if '--net' in sys.argv:
    only = sys.argv[sys.argv.index('--net') + 1]

names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
comp = ends[names[0]][2]
fp = pcb.footprints[comp]
grid = em.grid_of(fp)
print(f'destination {comp}: {len(grid.xs)} cols x {len(grid.ys)} rows, '
      f'pitch {grid.pitch_x:.3f} x {grid.pitch_y:.3f} (measured)')
LAYERS = ['F.Cu', 'B.Cu']
kids = {byname[n][0] for n in names}
obs = {}


def obs_for(nid, layer):
    if (nid, layer) not in obs:
        obs[(nid, layer)] = te.build_obstacles(pcb, nid, kids, layer)
    return obs[(nid, layer)]


tot = 0
for nm in names:
    if only and nm != only:
        continue
    nid = byname[nm][0]
    bx, by = ends[nm][1]
    pad = next(p for p in fp.pads
               if abs(p.global_x - bx) < 0.01 and abs(p.global_y - by) < 0.01)

    def clear(p, q, layer, _nid=nid):
        return obs_for(_nid, layer).seg_clear(p, q)

    def vclear(p, layer, _nid=nid):
        v = obs_for(_nid, layer).point_violation(
            p, pad=(te.VIA_SIZE - te.TRACK) / 2)
        return not (v and v[0] > 0)

    moves = em.enumerate_moves(pad, grid, LAYERS, clear, vclear)
    tot += len(moves)
    col, row = grid.col_of(bx), grid.row_of(by)
    print(f'\n{nm}  ball ({bx:.2f},{by:.2f}) col {col} row {row}  '
          f'-> {em.summarise(moves)}')
    if only:
        for m in moves:
            print('    ', m)
print(f'\ntotal moves across {len(names)} nets: {tot}')
