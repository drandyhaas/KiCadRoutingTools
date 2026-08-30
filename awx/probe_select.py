#!/usr/bin/env python3
"""What does the selector choose, and does it put the nets the old code
sent to the separate 'south river' builder onto a south/north edge?

usage: probe_select.py [K] [--west-only]
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402
import select_moves as sm  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') \
    else '21'
west_only = '--west-only' in sys.argv
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
kids = {byname[n][0] for n in names}
comps = {ends[n][2] for n in names}
pads = [p for c in comps for p in pcb.footprints[c].pads]
rows = sorted({round(p.global_y, 3) for p in pads})
river = {n for n in names if ends[n][0][1] > rows[-1] + 0.8}

obs = {}


def obs_for(nid, layer):
    if (nid, layer) not in obs:
        obs[(nid, layer)] = te.build_obstacles(pcb, nid, kids, layer)
    return obs[(nid, layer)]


LAYERS = ('F.Cu', 'B.Cu')
menu, launch = {}, {}
for nm in names:
    nid = byname[nm][0]
    ref = ends[nm][2]
    bx, by = ends[nm][1]
    fp = pcb.footprints[ref]
    grid = em.grid_of(fp)
    pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
              + (p.global_y - by) ** 2)

    def clear(p, q, layer, _n=nid):
        return obs_for(_n, layer).seg_clear(p, q)

    def vclear(p, layer, _n=nid):
        v = obs_for(_n, layer).point_violation(
            p, pad=(te.VIA_SIZE - te.TRACK) / 2)
        return not (v and v[0] > 0)

    menu[nm] = em.enumerate_moves(pad, grid, LAYERS, clear, vclear)
    launch[nm] = ends[nm][0]

grid0 = em.grid_of(pcb.footprints[ends[names[0]][2]])
import detect_buses as _db
_paths = _db.taut_paths(names, ends,
                        lambda nm: obs_for(byname[nm][0], 'F.Cu'))
groups = _db.cluster(names, _paths)
choice, unplaced = sm.select(menu, launch,
                             only_dirs={'left'} if west_only else None,
                             keep_out=grid0.bbox, buses=groups)
print(f'K={K}{"  (west-only)" if west_only else ""}: '
      + sm.summarise(choice))
if unplaced:
    print(f'  UNPLACED: {unplaced}')
# The corridor has to realise launch-order -> exit-order. Report that
# permutation for the chosen moves, against what the old westward-only
# assignment produced, since it is the corridor's actual workload.
import braid as _te
lefts = [n for n in names if choice.get(n) and choice[n].direction == 'left']
def perm_stats(group, keyfn):
    if len(group) < 2:
        return None
    launch_o = sorted(group, key=lambda n: ends[n][0][1])
    tgt = sorted(group, key=keyfn)
    tr = {n: i for i, n in enumerate(tgt)}
    ranks = [tr[n] for n in launch_o]
    inv = sum(1 for i in range(len(ranks))
              for j in range(i + 1, len(ranks)) if ranks[i] > ranks[j])
    lis = len(_te.lis_keep(ranks))
    return inv, lis, 2 * (len(group) - lis)

st = perm_stats(lefts, lambda n: choice[n].exit_pt[1])
if st:
    print(f'\ncorridor workload for the {len(lefts)} west-exit nets: '
          f'inversions {st[0]}, LIS {st[1]}, via floor {st[2]}')
st_all = perm_stats(names, lambda n: (ends[n][1][1], ends[n][1][0]))
if st_all:
    print(f'today (all {len(names)} nets, ball-y order):  '
          f'inversions {st_all[0]}, LIS {st_all[1]}, via floor {st_all[2]}')
print(f'\nnets the old code split into the separate builder ({len(river)}):')
for nm in sorted(river):
    m = choice.get(nm)
    print(f'  {nm:8s} {m if m else "UNPLACED"}')
